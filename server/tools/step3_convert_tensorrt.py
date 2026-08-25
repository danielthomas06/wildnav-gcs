#!/usr/bin/env python3
"""
STEP 3: Convert SuperPoint to TensorRT FP16
This gives the biggest single speedup (~3-5x faster inference)

Run: python3 step3_convert_tensorrt.py
Time: ~5-10 minutes on Jetson Nano (one-time cost)
"""

import os
import sys
import time
import torch
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))          # .../wild_opt/server/tools
_DEFAULT_BASE = os.path.dirname(os.path.dirname(_HERE))      # .../wild_opt
BASE = os.environ.get("WILDNAV_BASE", _DEFAULT_BASE)
WEIGHTS_DIR = os.path.join(BASE, "weights")
TRT_SAVE_PATH = os.path.join(WEIGHTS_DIR, "superpoint_trt.pth")
SP_WEIGHTS_PATH = os.path.join(WEIGHTS_DIR, "superpoint_v1.pth")

# Add models to path
sys.path.insert(0, BASE)
from models.superpoint import SuperPoint


# ────────────────────────────────────────────────────────────
# SuperPoint Encoder-only wrapper (encodable by TRT)
# TensorRT works best with fixed-shape, pure tensor I/O
# ────────────────────────────────────────────────────────────
class SuperPointEncoder(torch.nn.Module):
    """
    Wraps only the encoder + heads of SuperPoint.
    Returns raw score map and descriptor map (tensors only).
    Post-processing (NMS, keypoint extraction) stays in Python.
    """

    def __init__(self, superpoint_model):
        super().__init__()
        sp = superpoint_model
        self.relu  = sp.relu
        self.pool  = sp.pool
        self.conv1a = sp.conv1a; self.conv1b = sp.conv1b
        self.conv2a = sp.conv2a; self.conv2b = sp.conv2b
        self.conv3a = sp.conv3a; self.conv3b = sp.conv3b
        self.conv4a = sp.conv4a; self.conv4b = sp.conv4b
        self.convPa = sp.convPa; self.convPb = sp.convPb
        self.convDa = sp.convDa; self.convDb = sp.convDb

    def forward(self, image):
        """image: (1, 1, H, W) float32 tensor [0,1]"""
        x = self.relu(self.conv1a(image))
        x = self.relu(self.conv1b(x))
        x = self.pool(x)
        x = self.relu(self.conv2a(x))
        x = self.relu(self.conv2b(x))
        x = self.pool(x)
        x = self.relu(self.conv3a(x))
        x = self.relu(self.conv3b(x))
        x = self.pool(x)
        x = self.relu(self.conv4a(x))
        x = self.relu(self.conv4b(x))

        # Detector head
        cPa    = self.relu(self.convPa(x))
        scores = self.convPb(cPa)

        # Descriptor head
        cDa   = self.relu(self.convDa(x))
        descs = self.convDb(cDa)

        return scores, descs


def convert_to_tensorrt(image_height=240, image_width=320):
    print("=" * 55)
    print(" STEP 3: Converting SuperPoint → TensorRT FP16")
    print("=" * 55)

    # ── Check dependencies ────────────────────────────────────
    try:
        from torch2trt import torch2trt
        print("✓ torch2trt found")
    except ImportError:
        print("✗ torch2trt not installed!")
        print(f"  Run: cd {BASE}/torch2trt && python3 setup.py install")
        sys.exit(1)

    if not torch.cuda.is_available():
        print("✗ CUDA not available — TensorRT requires CUDA")
        sys.exit(1)

    # ── Load SuperPoint ───────────────────────────────────────
    print(f"\nLoading SuperPoint weights from:\n  {SP_WEIGHTS_PATH}")
    if not os.path.exists(SP_WEIGHTS_PATH):
        print("✗ Weights not found. Run step2_download_weights.py first.")
        sys.exit(1)

    config = {
        'descriptor_dim':    256,
        'nms_radius':        4,
        'keypoint_threshold': 0.005,
        'max_keypoints':     512,
        'remove_borders':    4,
    }
    sp_model = SuperPoint(config).eval().cuda()
    state    = torch.load(SP_WEIGHTS_PATH, map_location='cuda')
    sp_model.load_state_dict(state)
    print("✓ SuperPoint loaded")

    # ── Wrap encoder only ─────────────────────────────────────
    encoder = SuperPointEncoder(sp_model).eval().cuda()

    # ── Create dummy input at target resolution ───────────────
    dummy = torch.ones(
        (1, 1, image_height, image_width),
        dtype=torch.float32
    ).cuda()

    print(f"\nConverting to TensorRT FP16 at {image_width}×{image_height}...")
    print("This takes 5-10 minutes on Jetson Nano. Please wait...\n")

    t0 = time.time()
    try:
        encoder_trt = torch2trt(
            encoder,
            [dummy],
            fp16_mode=True,           # Key for Jetson: uses Tensor Cores
            max_workspace_size=1<<25, # 32 MB
        )
        elapsed = time.time() - t0
        print(f"✓ TensorRT conversion done in {elapsed:.1f}s")
    except Exception as e:
        print(f"✗ Conversion failed: {e}")
        sys.exit(1)

    # ── Save TRT model ────────────────────────────────────────
    torch.save(encoder_trt.state_dict(), TRT_SAVE_PATH)
    print(f"✓ Saved TRT model to:\n  {TRT_SAVE_PATH}")

    # ── Benchmark: PyTorch vs TensorRT ────────────────────────
    print("\n── Benchmark (100 runs) ──────────────────────────")
    test_img = torch.rand((1, 1, image_height, image_width)).cuda()

    with torch.no_grad():
        # Warm up
        for _ in range(5):
            encoder(test_img)
        torch.cuda.synchronize()

        t0 = time.time()
        for _ in range(100):
            encoder(test_img)
        torch.cuda.synchronize()
        pt_ms = (time.time() - t0) * 10  # ms per run

        # Warm up TRT
        for _ in range(5):
            encoder_trt(test_img)
        torch.cuda.synchronize()

        t0 = time.time()
        for _ in range(100):
            encoder_trt(test_img)
        torch.cuda.synchronize()
        trt_ms = (time.time() - t0) * 10

    speedup = pt_ms / trt_ms
    print(f"  PyTorch FP32:   {pt_ms:.1f} ms/frame")
    print(f"  TensorRT FP16:  {trt_ms:.1f} ms/frame")
    print(f"  Speedup:        {speedup:.1f}x")
    print()
    print("Next: Run python3 step4_run_wildnav.py")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--width",  type=int, default=320, help="Image width  (default 320)")
    parser.add_argument("--height", type=int, default=240, help="Image height (default 240)")
    args = parser.parse_args()
    convert_to_tensorrt(args.height, args.width)

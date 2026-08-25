#!/usr/bin/env bash
# =============================================================================
# WildNav one-shot installer (run on the drone / Jetson)
#
#   bash install.sh
#
# Idempotent — safe to re-run; each step skips if already done.
# What it does:
#   1. Installs miniconda if absent (aarch64-aware for Jetson)
#   2. Creates the `lightglue` conda env if absent
#   3. Installs system packages (apt) + python packages (pip, into the env)
#   4. Clones LightGlue + torch2trt into PERSISTENT paths (never /tmp)
#   5. Downloads SuperPoint weights + model definition
#   6. Runs TensorRT conversion at 1280x720 (step3_convert_tensorrt.py)
#   7. Installs the systemd service (wildnav-agent) + sudoers rule so the GUI
#      can toggle autostart without a password prompt
#   8. Writes config.json with the persistent drone name
# =============================================================================
set -euo pipefail

# ── Layout ───────────────────────────────────────────────────────────────────
# SRC = wherever this installer (and its server/ + web/ folders) was unpacked.
# BASE = permanent install target. Everything is copied SRC -> BASE, so the
# user can delete the download folder afterwards.
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE="${WILDNAV_BASE:-$HOME/wild_opt}"
SERVER_DIR="$BASE/server"
WEB_DIR="$BASE/web"
WEIGHTS_DIR="$BASE/weights"
MODELS_DIR="$BASE/models"
ENV_NAME="lightglue"
SERVICE_NAME="wildnav-agent"
DEFAULT_DRONE_NAME="${1:-drone-alpha}"

log()  { echo -e "\033[1;32m[install]\033[0m $*"; }
warn() { echo -e "\033[1;33m[warn]\033[0m $*"; }
die()  { echo -e "\033[1;31m[fail]\033[0m $*"; exit 1; }

# ── 0. Copy the payload (server/ + web/) into BASE ───────────────────────────
[ -d "$SRC_DIR/server" ] || die "server/ folder not found next to install.sh"
[ -d "$SRC_DIR/web" ]    || die "web/ folder not found next to install.sh"

# Integrity check — files sometimes get crossed when hand-copied (bash pasted
# into a .py or vice versa). Verify each payload file is what it claims to be
# BEFORE installing, and fail with a clear message instead of a confusing
# syntax error halfway through.
integrity_fail() {
    die "PAYLOAD CORRUPTED: $1
    A file's content doesn't match its type — this happens when files are
    copied by hand and mixed up. Fix: delete this folder and re-extract
    wildnav_gcs.zip fresh, then re-run install.sh. Do NOT hand-copy
    individual files between folders."
}
head -1 "$SRC_DIR/install.sh" | grep -q "bash" || integrity_fail "install.sh is not a bash script"
for pyf in server/drone_agent.py server/flight_engine.py server/nav_modes.py \
           server/discovery_hub.py server/tools/step3_convert_tensorrt.py; do
    [ -f "$SRC_DIR/$pyf" ] || integrity_fail "$pyf missing"
    # ast.parse is the authoritative validity test. (No grep heuristics — the
    # agent legitimately EMBEDS bash inside a python string for its setup
    # script, which a line-pattern check would false-positive on.)
    python3 -c "import ast; ast.parse(open('$SRC_DIR/$pyf').read())" 2>/dev/null \
        || integrity_fail "$pyf is not valid python"
done
grep -q "<html\|<!doctype" "$SRC_DIR/web/index.html" 2>/dev/null \
    || integrity_fail "web/index.html is not HTML"
log "payload integrity OK"

mkdir -p "$BASE" "$WEIGHTS_DIR" "$MODELS_DIR"
if [ "$SRC_DIR" != "$BASE" ]; then
    log "copying server/ + web/ -> $BASE"
    cp -r "$SRC_DIR/server" "$BASE/"
    cp -r "$SRC_DIR/web" "$BASE/"
    # strip dev junk if present
    rm -rf "$SERVER_DIR/__pycache__" "$SERVER_DIR/navigation_logs_siyi" \
           "$SERVER_DIR/uploads" 2>/dev/null || true
    # Verify the INSTALLED copies too (a failed/partial overwrite must not
    # survive silently). Force a byte-exact re-copy on mismatch.
    for pyf in server/drone_agent.py server/flight_engine.py \
               server/tools/step3_convert_tensorrt.py; do
        if ! python3 -c "import ast; ast.parse(open('$BASE/$pyf').read())" 2>/dev/null; then
            warn "installed $pyf invalid after copy — force re-copying"
            cat "$SRC_DIR/$pyf" > "$BASE/$pyf"
            python3 -c "import ast; ast.parse(open('$BASE/$pyf').read())" 2>/dev/null \
                || die "could not install a valid $pyf — check disk/permissions"
        fi
    done
else
    log "installer already running from $BASE — skipping copy"
fi

# ── 1. Miniconda ─────────────────────────────────────────────────────────────
find_conda() {
    for p in "$HOME/miniconda3" "$HOME/anaconda3" /opt/miniconda3 /opt/conda; do
        [ -f "$p/etc/profile.d/conda.sh" ] && { echo "$p"; return; }
    done
    echo ""
}
CONDA_BASE="$(find_conda)"
if [ -z "$CONDA_BASE" ]; then
    log "miniconda not found — installing to ~/miniconda3"
    ARCH="$(uname -m)"   # aarch64 on Jetson, x86_64 on PC
    URL="https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-${ARCH}.sh"
    curl -fsSL "$URL" -o /tmp/miniconda.sh || die "could not download miniconda"
    bash /tmp/miniconda.sh -b -p "$HOME/miniconda3"
    rm -f /tmp/miniconda.sh
    CONDA_BASE="$HOME/miniconda3"
else
    log "conda found at $CONDA_BASE"
fi
# shellcheck disable=SC1091
source "$CONDA_BASE/etc/profile.d/conda.sh"

# Newer conda releases require accepting channel Terms of Service before any
# env can be created non-interactively. Best-effort — older condas lack the
# `tos` subcommand entirely, hence `|| true`.
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main 2>/dev/null || true
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r    2>/dev/null || true

# ── 2. Env ───────────────────────────────────────────────────────────────────
# Python version policy:
#   Jetson : MATCH the system python3 (JetPack's TensorRT/torch bindings are
#            built for it — e.g. 3.8 on JetPack 5, 3.10 on JetPack 6).
#   other  : 3.10 (modern enough for current LightGlue/kornia, which need >=3.9)
is_jetson() { [ -f /etc/nv_tegra_release ] || dpkg -l 2>/dev/null | grep -q nvidia-l4t-core; }
if is_jetson; then
    PYVER="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
    log "Jetson detected — env python will match system python ($PYVER)"
else
    PYVER="3.10"
fi

env_python_ver() {
    "$CONDA_BASE/envs/$ENV_NAME/bin/python3" -c \
        'import sys; print("%d.%d" % sys.version_info[:2])' 2>/dev/null || echo "none"
}

if conda env list | grep -qE "^${ENV_NAME}[[:space:]]"; then
    CUR_VER="$(env_python_ver)"
    log "env '$ENV_NAME' exists (python $CUR_VER)"
    # A stale env with python <3.9 can't run current LightGlue/kornia. Only
    # recreate if lightglue ISN'T already working there — never touch a
    # functioning install.
    if [ "$CUR_VER" != "$PYVER" ] && \
       ! "$CONDA_BASE/envs/$ENV_NAME/bin/python3" -c "import lightglue" 2>/dev/null; then
        case "$CUR_VER" in
            3.[0-8])
                warn "env python $CUR_VER is too old for LightGlue (needs >=3.9)"
                warn "recreating '$ENV_NAME' with python $PYVER"
                conda env remove -y -n "$ENV_NAME"
                conda create -y -n "$ENV_NAME" "python=$PYVER" || \
                    conda create -y -n "$ENV_NAME" -c conda-forge "python=$PYVER"
                ;;
        esac
    fi
else
    log "creating env '$ENV_NAME' (python $PYVER)"
    conda create -y -n "$ENV_NAME" "python=$PYVER" || {
        warn "default channels failed — retrying via conda-forge"
        conda create -y -n "$ENV_NAME" -c conda-forge "python=$PYVER"
    }
fi
conda activate "$ENV_NAME"
PY="$CONDA_BASE/envs/$ENV_NAME/bin/python3"

# ── 3. Packages ──────────────────────────────────────────────────────────────
log "apt packages (needs sudo): ffmpeg git netcat curl + build headers"
sudo apt-get update -qq || warn "apt update failed — continuing"
sudo apt-get install -y -qq ffmpeg git netcat-openbsd curl \
    libxml2-dev libxslt1-dev zlib1g-dev build-essential \
    || warn "some apt installs failed"

# ── 3a. pip packages — installed ONE BY ONE so a single failure never blocks
#        the rest, with the package name logged so long installs are visible
#        (a silent -q pip looks frozen while compiling on a Nano).
log "pip packages into '$ENV_NAME'"
"$PY" -m pip install -q --upgrade pip

# Clean orphans from any earlier failed run: torchvision without torch makes
# pip's resolver print scary (but blocking-looking) errors on every install.
if "$PY" -m pip show torchvision >/dev/null 2>&1 && \
   ! "$PY" -c "import torch" 2>/dev/null; then
    warn "removing orphaned torchvision (torch is absent)"
    "$PY" -m pip uninstall -y torchvision 2>/dev/null || true
fi

# lxml first: pymavlink depends on it, and on platforms without a prebuilt
# wheel it must compile against the libxml2/libxslt headers installed above
# (compiling takes ~10 min on a Nano — the log line shows it's working).
log " -> lxml (may compile from source — this is slow but normal)"
"$PY" -m pip install --prefer-binary lxml || warn "lxml install failed — pymavlink will fail too"

# opencv-python-headless: same cv2 API, no GUI deps — right choice for a
# headless drone, and it has binary wheels for aarch64. NEVER build cv2 from
# source on a Nano (hours).
PIP_PKGS=(fastapi "uvicorn[standard]" zeroconf python-multipart pillow
          opencv-python-headless kornia matplotlib pymavlink pandas pyproj "numpy<2"
          paho-mqtt)
FAILED_PKGS=()
for pkg in "${PIP_PKGS[@]}"; do
    log " -> $pkg"
    "$PY" -m pip install --prefer-binary "$pkg" || FAILED_PKGS+=("$pkg")
done
[ ${#FAILED_PKGS[@]} -gt 0 ] && warn "failed pip packages: ${FAILED_PKGS[*]}"

# ── 3b. rasterio — needs GDAL. Try in order of least→most invasive:
#        (1) prebuilt wheel only, (2) conda-forge binary, (3) apt GDAL dev
#        headers then source build. Works on x86_64 and aarch64 alike.
if ! "$PY" -c "import rasterio" 2>/dev/null; then
    log "installing rasterio (wheel -> conda-forge -> GDAL source build)"
    "$PY" -m pip install -q --only-binary=:all: rasterio 2>/dev/null \
    || conda install -y -q -n "$ENV_NAME" -c conda-forge rasterio 2>/dev/null \
    || {
        warn "no rasterio binary for this platform — building against system GDAL"
        sudo apt-get install -y -qq libgdal-dev gdal-bin || true
        if command -v gdal-config >/dev/null; then
            GDAL_VERSION="$(gdal-config --version)"
            "$PY" -m pip install -q "rasterio" \
                --no-binary rasterio \
                --global-option=build_ext 2>/dev/null \
            || GDAL_CONFIG="$(command -v gdal-config)" "$PY" -m pip install -q rasterio \
            || warn "rasterio install failed — map upload validation will not work until fixed"
        else
            warn "gdal-config still missing — rasterio skipped"
        fi
    }
fi
"$PY" -c "import rasterio" 2>/dev/null && log "rasterio OK" || warn "rasterio NOT available"

# ── 3c. torch — platform-aware, CUDA-VERIFIED. The trap: PyPI ships CPU-only
#   aarch64 torch wheels, and any resolver that can see PyPI may pick one and
#   "succeed" while silently killing CUDA. So on Jetson: each attempt is
#   verified with torch.cuda.is_available(); CPU builds are uninstalled and
#   the next index is tried.
torch_cuda_ok() {
    "$PY" -c "import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)" 2>/dev/null
}
if is_jetson && "$PY" -c "import torch" 2>/dev/null && ! torch_cuda_ok; then
    warn "existing torch in env is CPU-only — removing it (Jetson needs a CUDA build)"
    "$PY" -m pip uninstall -q -y torch torchvision 2>/dev/null || true
fi
if ! "$PY" -c "import torch" 2>/dev/null; then
    ARCH="$(uname -m)"
    if [ "$ARCH" = "x86_64" ]; then
        log "installing torch + torchvision (x86_64 wheels)"
        "$PY" -m pip install --prefer-binary torch torchvision || warn "torch install failed"
    elif is_jetson; then
        log "Jetson detected — installing CUDA torch (exclusive index; PyPI CPU wheels locked out)"
        CPTAG="cp${PYVER//./}"     # e.g. cp310
        TORCH_OK=0
        torch_deps() {  # torch's pure-python deps, safe to take from PyPI
            "$PY" -m pip install --prefer-binary -q filelock typing-extensions \
                sympy networkx jinja2 fsspec 2>/dev/null || true
        }
        # Tier 1: jetson-ai-lab PEP503 indexes. --index-url EXCLUSIVE (no pypi
        # extra!) so pip's newest-version-wins resolver can only see CUDA
        # wheels — a PyPI CPU wheel can never outrank them. --no-deps because
        # the index may not mirror every dep (installed separately above).
        # Short retries/timeout so an unreachable host costs seconds.
        for IDX in \
            "https://pypi.jetson-ai-lab.dev/jp6/cu126" \
            "https://pypi.jetson-ai-lab.dev/jp6/cu122" \
            "https://pypi.jetson-ai-lab.dev/jp5/cu114" ; do
            log " -> trying index $IDX"
            "$PY" -m pip install --no-deps --prefer-binary \
                --retries 1 --timeout 15 \
                torch --index-url "$IDX" || continue
            torch_deps
            if torch_cuda_ok; then
                log "CUDA torch installed from $IDX"
                "$PY" -m pip install --no-deps --prefer-binary --retries 1 \
                    --timeout 15 torchvision --index-url "$IDX" || \
                    warn "torchvision unavailable from index (optional)"
                TORCH_OK=1; break
            fi
            "$PY" -m pip uninstall -y torch torchvision 2>/dev/null || true
        done
        # Tier 2: NVIDIA's official redist server — scrape for the wheel that
        # matches this env's python tag and install it by direct URL.
        if [ "$TORCH_OK" = 0 ]; then
            for JPV in v61 v60 v512 v511; do
                RURL="https://developer.download.nvidia.com/compute/redist/jp/$JPV/pytorch/"
                WHL="$(curl -fsSL --max-time 20 "$RURL" 2>/dev/null \
                       | grep -oE "torch-[^\"']+${CPTAG}-${CPTAG}[^\"']*\.whl" \
                       | head -1)"
                [ -z "$WHL" ] && continue
                log " -> trying NVIDIA redist $JPV: $WHL"
                "$PY" -m pip install --no-deps --prefer-binary "$RURL$WHL" || continue
                torch_deps
                if torch_cuda_ok; then
                    log "CUDA torch installed from NVIDIA redist $JPV"
                    TORCH_OK=1; break
                fi
                "$PY" -m pip uninstall -y torch 2>/dev/null || true
            done
        fi
        if [ "$TORCH_OK" = 0 ]; then
            warn "could not auto-install CUDA torch. Install NVIDIA's wheel for"
            warn "your JetPack manually (must match Python $("$PY" -V | cut -d' ' -f2)):"
            warn "  https://developer.nvidia.com/embedded/downloads#?search=pytorch"
            warn "  or browse https://pypi.jetson-ai-lab.dev"
            warn "Then re-run this installer."
        fi
    else
        log "generic aarch64 — installing CPU torch wheel"
        "$PY" -m pip install --prefer-binary torch torchvision || warn "torch install failed"
    fi
fi
if "$PY" -c "import torch" 2>/dev/null; then
    if torch_cuda_ok; then
        log "torch OK (CUDA available)"
    else
        warn "torch installed but CUDA NOT available — flight vision + TRT need CUDA"
    fi
    # torch <-> numpy ABI check. Jetson torch wheels are typically built
    # against numpy 1.x; a numpy 2.x in the env makes every tensor<->array
    # conversion fail with the cryptic "Numpy is not available". Detect and
    # downgrade automatically.
    if ! "$PY" -c "import torch, numpy; torch.from_numpy(numpy.zeros(2))" 2>/dev/null; then
        warn "torch<->numpy interop broken (numpy 2.x vs torch built for 1.x) — downgrading numpy"
        "$PY" -m pip install --prefer-binary "numpy<2"
        "$PY" -c "import torch, numpy; torch.from_numpy(numpy.zeros(2))" 2>/dev/null \
            && log "torch<->numpy interop OK after downgrade" \
            || warn "interop still broken — check numpy/torch versions manually"
    fi
fi

# ── 4. Repos (persistent — never /tmp) ───────────────────────────────────────
if [ ! -d "$BASE/LightGlue" ]; then
    log "cloning LightGlue -> $BASE/LightGlue"
    git clone -q https://github.com/cvg/LightGlue "$BASE/LightGlue"
fi
if ! "$PY" -c "import lightglue" 2>/dev/null; then
    log "installing LightGlue editable (--no-deps: NEVER let it pull its own torch)"
    # CRITICAL: without --no-deps, LightGlue's dependency resolution can
    # install a CPU-only torch wheel OVER a working CUDA build. We install
    # its deps ourselves above.
    (cd "$BASE/LightGlue" && "$PY" -m pip install -e . --no-deps) || {
        warn "LightGlue install FAILED. Most common cause: env python too old"
        warn "(LightGlue needs >=3.9; this env has $("$PY" -V)). Re-run the"
        warn "installer — it recreates old envs automatically when needed."
    }
fi
# LightGlue's __init__ imports ALL extractors (ALIKED, DISK, SIFT...), some of
# which need torchvision/kornia. We only use SuperPoint + LightGlue, so guard
# the optional imports — a missing torchvision must not kill the package.
# Idempotent (marker-guarded); a re-clone gets re-patched.
LG_INIT="$BASE/LightGlue/lightglue/__init__.py"
if [ -f "$LG_INIT" ] && ! grep -q "WILDNAV-GUARD" "$LG_INIT"; then
    log "guarding optional extractor imports in lightglue/__init__.py"
    "$PY" - "$LG_INIT" <<'PYEOF'
import sys, re
p = sys.argv[1]
src = open(p).read().splitlines()
out = []
for ln in src:
    m = re.match(r"from \.(\w+) import (\w+)", ln.strip())
    if m and m.group(1) not in ("lightglue", "superpoint", "utils"):
        out.append("try:  # WILDNAV-GUARD: optional extractor, may lack torchvision/kornia")
        out.append("    " + ln.strip())
        out.append("except Exception:")
        out.append(f"    {m.group(2)} = None")
    else:
        out.append(ln)
open(p, "w").write("\n".join(out) + "\n")
print("patched", p)
PYEOF
fi

if [ ! -d "$BASE/torch2trt" ]; then
    log "cloning torch2trt -> $BASE/torch2trt"
    git clone -q https://github.com/NVIDIA-AI-IOT/torch2trt "$BASE/torch2trt"
fi

# ── 4b. TensorRT — check, and INSTALL if missing ─────────────────────────────
#   x86_64 : NVIDIA publishes tensorrt wheels on PyPI -> plain pip install
#   Jetson : TensorRT ships with JetPack for the SYSTEM python; a conda env
#            can't see it. We apt-install the bindings if absent, then bridge
#            them into the env by symlinking the tensorrt packages into the
#            env's site-packages. Requires env python version == system python
#            version (both 3.8 on JetPack 5) — we verify and warn otherwise.
if ! "$PY" -c "import tensorrt" 2>/dev/null; then
    ARCH="$(uname -m)"
    if is_jetson; then
        log "TensorRT bindings not in env — bridging from JetPack"
        # Ensure system bindings exist. Package names differ across JetPack
        # releases — try the known candidates; all come from NVIDIA's L4T apt
        # repo which JetPack systems have preconfigured.
        if ! python3 -c "import tensorrt" 2>/dev/null; then
            for TP in python3-libnvinfer python3-libnvinfer-dev \
                      nvidia-tensorrt tensorrt python3-tensorrt; do
                sudo apt-get install -y -qq "$TP" 2>/dev/null || true
            done
        fi
        if ! python3 -c "import tensorrt" 2>/dev/null; then
            warn "system python has no tensorrt either — your JetPack install may"
            warn "be missing TensorRT. Try: sudo apt install nvidia-jetpack"
        fi
        SYS_PYV="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
        ENV_PYV="$("$PY" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
        if [ "$SYS_PYV" != "$ENV_PYV" ]; then
            warn "system python ($SYS_PYV) != env python ($ENV_PYV) —"
            warn "JetPack TensorRT bindings can't be bridged across versions."
        else
            ENV_SITE="$("$PY" -c 'import site; print(site.getsitepackages()[0])')"
            BRIDGED=0
            for d in "/usr/lib/python3/dist-packages"/tensorrt* \
                     "/usr/lib/python${SYS_PYV}/dist-packages"/tensorrt*; do
                if [ -e "$d" ]; then
                    ln -sfn "$d" "$ENV_SITE/$(basename "$d")"
                    BRIDGED=1
                fi
            done
            [ "$BRIDGED" = 1 ] && log "bridged JetPack TensorRT into env" \
                               || warn "no system tensorrt packages found to bridge"
        fi
    elif [ "$ARCH" = "x86_64" ]; then
        log "installing TensorRT wheel (x86_64)"
        "$PY" -m pip install -q tensorrt || warn "tensorrt pip install failed (needs NVIDIA GPU + CUDA)"
    else
        warn "no TensorRT path for generic $ARCH — TRT conversion unavailable"
    fi
fi
"$PY" -c "import tensorrt" 2>/dev/null && log "TensorRT OK" \
    || warn "TensorRT NOT available — flight still works; TRT speedup disabled"

# ── 4c. torch2trt (needs TensorRT above) ─────────────────────────────────────
if ! "$PY" -c "import torch2trt" 2>/dev/null; then
    if "$PY" -c "import tensorrt" 2>/dev/null; then
        log "installing torch2trt"
        (cd "$BASE/torch2trt" && "$PY" setup.py install -q) || \
            warn "torch2trt install failed"
    else
        warn "TensorRT python bindings not found — torch2trt skipped."
    fi
fi

# ── 5. SuperPoint weights + model def ────────────────────────────────────────
SP_PY="$MODELS_DIR/superpoint.py"
SP_W="$WEIGHTS_DIR/superpoint_v1.pth"
[ -f "$MODELS_DIR/__init__.py" ] || touch "$MODELS_DIR/__init__.py"
if [ ! -f "$SP_PY" ]; then
    log "downloading models/superpoint.py"
    curl -fsSL "https://raw.githubusercontent.com/magicleap/SuperGluePretrainedNetwork/master/models/superpoint.py" -o "$SP_PY"
fi
if [ ! -f "$SP_W" ]; then
    log "downloading superpoint_v1.pth"
    curl -fsSL "https://github.com/magicleap/SuperGluePretrainedNetwork/raw/master/models/weights/superpoint_v1.pth" -o "$SP_W"
fi
# magicleap's superpoint.py loads weights from models/weights/superpoint_v1.pth
# RELATIVE TO ITSELF (hardcoded inside the class) — bridge with a symlink so
# both the canonical weights dir and the model's expectation are satisfied.
mkdir -p "$MODELS_DIR/weights"
ln -sfn "$SP_W" "$MODELS_DIR/weights/superpoint_v1.pth"

# ── 6. TensorRT conversion @ 1280x720 ───────────────────────────────────────
TRT_OUT="$WEIGHTS_DIR/superpoint_trt.pth"
if [ -f "$TRT_OUT" ]; then
    log "TRT model already exists ($TRT_OUT) — skipping conversion"
else
    if "$PY" -c "import torch2trt, torch; assert torch.cuda.is_available()" 2>/dev/null; then
        log "running TensorRT conversion (1280x720) — 5-10 min on Jetson"
        (cd "$SERVER_DIR/tools" && \
         WILDNAV_BASE="$BASE" "$PY" step3_convert_tensorrt.py --width 1280 --height 720) \
            || warn "TRT conversion failed — re-run later from the GUI settings panel"
    else
        warn "skipping TRT conversion (torch2trt or CUDA unavailable)"
    fi
fi

# ── 7. systemd service + sudoers ────────────────────────────────────────────
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"
log "installing systemd service $SERVICE_NAME (needs sudo)"
sudo tee "$SERVICE_FILE" >/dev/null <<EOF
[Unit]
Description=WildNav Drone Agent
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$SERVER_DIR
# Optional ('-' prefix): loads cloud_relay.env if present, so the cloud relay
# (see cloud_relay.py) comes up under the autostart service too, not just a
# manually-sourced shell. Absent file = cloud relay stays disabled, same as
# running without it manually.
EnvironmentFile=-$SERVER_DIR/cloud_relay.env
ExecStart=$CONDA_BASE/envs/$ENV_NAME/bin/python3 $SERVER_DIR/drone_agent.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload

# Passwordless toggle of ONLY this service, so the GUI autostart switch works.
SUDOERS_FILE="/etc/sudoers.d/wildnav-agent"
sudo tee "$SUDOERS_FILE" >/dev/null <<EOF
$USER ALL=(root) NOPASSWD: /usr/bin/systemctl enable $SERVICE_NAME, /usr/bin/systemctl disable $SERVICE_NAME, /usr/bin/systemctl start $SERVICE_NAME, /usr/bin/systemctl stop $SERVICE_NAME, /usr/bin/systemctl is-enabled $SERVICE_NAME
EOF
sudo chmod 440 "$SUDOERS_FILE"

# ── 8. config.json ───────────────────────────────────────────────────────────
CONFIG="$SERVER_DIR/config.json"
if [ ! -f "$CONFIG" ]; then
    log "writing config.json (drone name: $DEFAULT_DRONE_NAME)"
    cat > "$CONFIG" <<EOF
{
  "drone_name": "$DEFAULT_DRONE_NAME"
}
EOF
else
    log "config.json exists — keeping current name"
fi

echo
log "── Install health report ─────────────────────────────"
check() {  # check <label> <python import expr>
    if "$PY" -c "$2" 2>/dev/null; then
        echo -e "  \033[1;32m✓\033[0m $1"
    else
        echo -e "  \033[1;31m✗\033[0m $1"
    fi
}
check "python env ($ENV_NAME)"          "import sys"
check "fastapi + uvicorn (agent/GUI)"   "import fastapi, uvicorn"
check "zeroconf (discovery)"            "import zeroconf"
check "paho-mqtt (cloud relay)"         "import paho.mqtt.client"
check "rasterio (GeoTIFF maps)"         "import rasterio"
check "opencv"                          "import cv2"
check "torch"                           "import torch"
check "torch CUDA"                      "import torch,sys; sys.exit(0 if torch.cuda.is_available() else 1)"
check "torch<->numpy interop"           "import torch,numpy; torch.from_numpy(numpy.zeros(2))"
check "lightglue (localisation)"        "import lightglue"
"$PY" -c "import lightglue" 2>/dev/null || \
    echo "      reason: $("$PY" -c 'import lightglue' 2>&1 | tail -1)"
check "tensorrt (python bindings)"      "import tensorrt"
check "torch2trt (TRT conversion)"      "import torch2trt"
check "pymavlink (flight control)"      "import pymavlink"
[ -f "$WEIGHTS_DIR/superpoint_v1.pth" ] \
    && echo -e "  \033[1;32m✓\033[0m superpoint weights" \
    || echo -e "  \033[1;31m✗\033[0m superpoint weights"
[ -f "$WEIGHTS_DIR/superpoint_trt.pth" ] \
    && echo -e "  \033[1;32m✓\033[0m TRT engine (1280x720)" \
    || echo -e "  \033[1;33m○\033[0m TRT engine — build later from GUI settings"
echo
log "Anything marked ✗ above: fix it and RE-RUN this installer — it is"
log "idempotent and only redoes the missing pieces."
echo
log "DONE. Next steps:"
echo "  Start now:            sudo systemctl start $SERVICE_NAME"
echo "  Autostart on boot:    sudo systemctl enable $SERVICE_NAME   (or toggle in GUI)"
echo "  Manual run:           conda activate $ENV_NAME && python3 $SERVER_DIR/drone_agent.py"
echo "  GUI:                  http://<this-drone-ip>:8000"

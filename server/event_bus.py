"""
event_bus.py — tiny in-process pub/sub fan-out.

RUNNER.event_q (mp.Queue) and TRT.log_q are single-consumer: whoever calls
drain_events()/drain() first gets the items, and the other misses them. That
was fine when the only consumer was the local /ws handler, but cloud_relay.py
needs to see the same events too. Rather than have two places race to drain
the same source, drone_agent.py now drains them in exactly one place (its
central pump loop) and publishes each item here — the WebSocket handler and
cloud_relay each get their own subscriber queue and see everything.
"""
import queue
import threading


class EventBus:
    def __init__(self):
        self._subs = []
        self._lock = threading.Lock()

    def subscribe(self, maxsize=2000):
        q = queue.Queue(maxsize=maxsize)
        with self._lock:
            self._subs.append(q)
        return q

    def unsubscribe(self, q):
        with self._lock:
            if q in self._subs:
                self._subs.remove(q)

    def publish(self, event):
        with self._lock:
            subs = list(self._subs)
        for q in subs:
            try:
                q.put_nowait(event)
            except queue.Full:
                # Slow consumer (e.g. a stalled cloud connection) — drop the
                # oldest queued item rather than block every other subscriber.
                try:
                    q.get_nowait()
                except queue.Empty:
                    pass
                try:
                    q.put_nowait(event)
                except queue.Full:
                    pass


bus = EventBus()

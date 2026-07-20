"""Background polling of motor parameters for the live table and oscilloscope.

Type-17 reads are request/response, so a poller thread walks its index list in
round-robin and hands each decoded sample to a callback. Feedback frames
(type 2) arrive on their own via the bus listener and do not need polling -
enable active reporting instead for the fastest position/velocity/torque
stream the firmware offers.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable

from .motor import Motor

log = logging.getLogger(__name__)


@dataclass
class Sample:
    motor_id: int
    index: int
    value: object
    timestamp: float


class ParamPoller:
    """Round-robins a set of parameter indices on one motor."""

    def __init__(self, motor: Motor, callback: Callable[[Sample], None],
                 indices: Iterable[int] = (), interval: float = 0.05,
                 read_timeout: float = 0.08):
        self.motor = motor
        self.callback = callback
        self.interval = interval
        self.read_timeout = read_timeout

        self._indices = list(indices)
        self._indices_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = threading.Event()

        self.ok_count = 0
        self.timeout_count = 0

    # -- configuration ----------------------------------------------------

    def set_indices(self, indices: Iterable[int]) -> None:
        with self._indices_lock:
            self._indices = list(indices)

    def get_indices(self) -> list[int]:
        with self._indices_lock:
            return list(self._indices)

    def set_interval(self, seconds: float) -> None:
        self.interval = max(0.001, seconds)

    # -- lifecycle --------------------------------------------------------

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    def start(self) -> None:
        if self._running.is_set():
            return
        self._running.set()
        self._thread = threading.Thread(
            target=self._loop, name=f"poll-{self.motor.motor_id}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _loop(self) -> None:
        while self._running.is_set():
            indices = self.get_indices()
            if not indices:
                time.sleep(0.05)
                continue

            cycle_start = time.perf_counter()
            for index in indices:
                if not self._running.is_set():
                    break
                try:
                    value = self.motor.read(index, self.read_timeout)
                except Exception as exc:
                    log.debug("poll read 0x%04X failed: %s", index, exc)
                    value = None
                if value is None:
                    self.timeout_count += 1
                    continue
                self.ok_count += 1
                try:
                    self.callback(Sample(self.motor.motor_id, index, value,
                                         time.time()))
                except Exception:
                    log.exception("poller callback failed")

            elapsed = time.perf_counter() - cycle_start
            remaining = self.interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

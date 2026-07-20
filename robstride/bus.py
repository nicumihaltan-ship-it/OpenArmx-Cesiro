"""CAN transport layer built on python-can.

The same PEAK PCAN-USB Pro FD is reached through different backends depending
on the platform:

* **Windows** - the ``pcan`` backend, which loads PEAK's ``PCANBasic.dll``.
  Channels are named ``PCAN_USBBUS1``, ``PCAN_USBBUS2``, ...
* **Linux** - the ``peak_usb`` kernel module presents the adapter as ordinary
  SocketCAN interfaces, so the ``socketcan`` backend is used and channels are
  named ``can0``, ``can1``, ... Bring them up before connecting:

      sudo ip link set can0 up type can bitrate 1000000

  (python-can cannot set the bitrate on an existing SocketCAN interface; the
  ``ip link`` command is what actually configures it.)

One :class:`CanLink` owns one channel: a receive thread, a trace ring buffer,
and a small request/response table so type-17 parameter reads can be awaited
synchronously.
"""

from __future__ import annotations

import logging
import platform
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import can

from . import protocol as proto

log = logging.getLogger(__name__)

#: Bitrates the motors support, keyed by the type-23 F_CMD code.
BITRATES = {1: 1_000_000, 2: 500_000, 3: 250_000, 4: 125_000}

PCAN_CHANNELS = [f"PCAN_USBBUS{i}" for i in range(1, 17)]
SOCKETCAN_CHANNELS = [f"can{i}" for i in range(0, 8)]

#: Backends worth offering in the UI, in preference order per platform.
INTERFACES = ["pcan", "socketcan", "virtual"]


def is_linux() -> bool:
    return platform.system() == "Linux"


def default_interface() -> str:
    """The backend that reaches a PEAK adapter on this platform."""
    return "socketcan" if is_linux() else "pcan"


def default_channels() -> list[str]:
    """Candidate channel names for the default backend."""
    return SOCKETCAN_CHANNELS if is_linux() else PCAN_CHANNELS


def channel_candidates(interface: str) -> list[str]:
    if interface == "socketcan":
        return SOCKETCAN_CHANNELS
    if interface == "pcan":
        return PCAN_CHANNELS
    if interface == "virtual":
        return ["vcan0", "vcan1"]
    return []


@dataclass
class TraceEntry:
    timestamp: float
    direction: str          # "TX" or "RX"
    can_id: int
    data: bytes
    comm_type: int
    data2: int
    dest: int


class CanError(RuntimeError):
    pass


class CanLink:
    """A single CAN channel with RobStride framing on top."""

    def __init__(self, channel: str, bitrate: int = 1_000_000,
                 interface: str | None = None,
                 host_id: int = proto.DEFAULT_HOST_ID,
                 trace_size: int = 20000):
        interface = interface or default_interface()
        self.channel = channel
        self.bitrate = bitrate
        self.interface = interface
        self.host_id = host_id

        self._bus: can.BusABC | None = None
        self._rx_thread: threading.Thread | None = None
        self._running = threading.Event()

        self.trace: deque[TraceEntry] = deque(maxlen=trace_size)
        self._trace_lock = threading.Lock()

        # comm_type -> list of callbacks(can_id, data)
        self._listeners: dict[int, list[Callable[[int, bytes], None]]] = {}
        self._listener_lock = threading.Lock()

        # (motor_id, index) -> Event/result, for awaiting type-17 replies
        self._pending: dict[tuple[int, int], list] = {}
        self._pending_lock = threading.Lock()

        self._tx_lock = threading.Lock()
        self.tx_count = 0
        self.rx_count = 0
        self.error_count = 0

    # -- lifecycle --------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    def open(self) -> None:
        if self._bus is not None:
            return
        kwargs = {"interface": self.interface, "channel": self.channel}
        if self.interface != "socketcan":
            # SocketCAN takes its bitrate from `ip link`, not from python-can;
            # passing one here is rejected by some kernel/driver combinations.
            kwargs["bitrate"] = self.bitrate
        try:
            self._bus = can.Bus(**kwargs)
        except Exception as exc:  # python-can raises a wide variety here
            raise CanError(f"Could not open {self.channel}: {exc}\n\n"
                           + self._open_hint()) from exc

        self._running.set()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name=f"rx-{self.channel}", daemon=True)
        self._rx_thread.start()
        log.info("Opened %s at %d bit/s", self.channel, self.bitrate)

    def _open_hint(self) -> str:
        """Platform-specific advice for the most common open failures."""
        if self.interface == "socketcan":
            return (f"On Linux the PEAK adapter appears via the peak_usb kernel "
                    f"module as a SocketCAN interface. Bring it up first:\n"
                    f"    sudo ip link set {self.channel} up type can "
                    f"bitrate {self.bitrate}\n"
                    f"Check it exists with:  ip -details link show {self.channel}")
        return ("On Windows this needs PEAK's driver package, which provides "
                "PCANBasic.dll. Install it from "
                "https://www.peak-system.com/Drivers.523.0.html and confirm the "
                "adapter is connected.")

    def close(self) -> None:
        self._running.clear()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
            self._rx_thread = None
        if self._bus is not None:
            try:
                self._bus.shutdown()
            finally:
                self._bus = None
        log.info("Closed %s", self.channel)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # -- transmit ---------------------------------------------------------

    def send(self, frame: proto.Frame) -> None:
        if self._bus is None:
            raise CanError("Channel is not open")
        msg = can.Message(arbitration_id=frame.can_id, data=frame.data,
                          is_extended_id=True)
        with self._tx_lock:
            try:
                self._bus.send(msg, timeout=0.5)
            except Exception as exc:
                self.error_count += 1
                raise CanError(f"Send failed on {self.channel}: {exc}") from exc
            self.tx_count += 1
        self._record("TX", frame.can_id, frame.data)

    def send_raw(self, can_id: int, data: bytes, extended: bool = True) -> None:
        """Escape hatch for the trace view's manual-transmit box."""
        if self._bus is None:
            raise CanError("Channel is not open")
        msg = can.Message(arbitration_id=can_id, data=bytes(data)[:8],
                          is_extended_id=extended)
        with self._tx_lock:
            self._bus.send(msg, timeout=0.5)
            self.tx_count += 1
        self._record("TX", can_id, bytes(data))

    # -- receive ----------------------------------------------------------

    def _rx_loop(self) -> None:
        while self._running.is_set():
            try:
                msg = self._bus.recv(timeout=0.1)
            except Exception as exc:
                if self._running.is_set():
                    log.warning("recv error on %s: %s", self.channel, exc)
                    self.error_count += 1
                continue
            if msg is None:
                continue
            self.rx_count += 1
            data = bytes(msg.data)
            self._record("RX", msg.arbitration_id, data)
            self._dispatch(msg.arbitration_id, data)

    def _record(self, direction: str, can_id: int, data: bytes) -> None:
        comm_type, data2, dest = proto.unpack_id(can_id)
        entry = TraceEntry(time.time(), direction, can_id, data,
                           comm_type, data2, dest)
        with self._trace_lock:
            self.trace.append(entry)

    def _dispatch(self, can_id: int, data: bytes) -> None:
        comm_type, data2, _ = proto.unpack_id(can_id)

        # Satisfy any awaiting parameter read.
        if comm_type == proto.CommType.PARAM_READ and len(data) >= 8:
            reply = proto.decode_param_reply(can_id, data)
            key = (reply.motor_id, reply.index)
            with self._pending_lock:
                slot = self._pending.get(key)
                if slot is not None:
                    slot[0] = reply
                    slot[1].set()

        with self._listener_lock:
            callbacks = list(self._listeners.get(comm_type, ()))
            callbacks += list(self._listeners.get(-1, ()))  # -1 == wildcard
        for cb in callbacks:
            try:
                cb(can_id, data)
            except Exception:
                log.exception("listener for comm_type %d failed", comm_type)

    def add_listener(self, comm_type: int,
                     callback: Callable[[int, bytes], None]) -> None:
        """Register a callback. ``comm_type=-1`` receives every frame."""
        with self._listener_lock:
            self._listeners.setdefault(comm_type, []).append(callback)

    def remove_listener(self, comm_type: int, callback) -> None:
        with self._listener_lock:
            try:
                self._listeners.get(comm_type, []).remove(callback)
            except ValueError:
                pass

    # -- request / response ----------------------------------------------

    def read_param_raw(self, motor_id: int, index: int,
                       timeout: float = 0.25) -> proto.ParamReply | None:
        """Send a type-17 read and block until the reply arrives (or times out)."""
        key = (motor_id, index)
        slot = [None, threading.Event()]
        with self._pending_lock:
            self._pending[key] = slot
        try:
            self.send(proto.param_read(motor_id, index, self.host_id))
            if slot[1].wait(timeout):
                return slot[0]
            return None
        finally:
            with self._pending_lock:
                self._pending.pop(key, None)

    def snapshot_trace(self) -> list[TraceEntry]:
        with self._trace_lock:
            return list(self.trace)

    def clear_trace(self) -> None:
        with self._trace_lock:
            self.trace.clear()


def available_channels(interface: str | None = None) -> list[str]:
    """Probe which channels can actually be reached right now.

    On Linux this also lists SocketCAN interfaces that exist but are still
    down, since those are the ones the user most likely needs to bring up.
    """
    interface = interface or default_interface()
    found: list[str] = []
    try:
        for entry in can.detect_available_configs(interface):
            channel = entry.get("channel")
            if channel:
                found.append(str(channel))
    except Exception as exc:
        log.debug("%s autodetect unavailable: %s", interface, exc)

    if interface == "socketcan":
        # detect_available_configs only reports interfaces that are already up.
        try:
            for path in sorted(Path("/sys/class/net").iterdir()):
                name = path.name
                if (name.startswith("can") or name.startswith("vcan")) \
                        and name not in found:
                    found.append(name)
        except OSError:
            pass
    return found


#: Kept for callers that predate the multi-backend support.
available_pcan_channels = available_channels

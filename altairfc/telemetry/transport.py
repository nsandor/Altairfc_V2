from __future__ import annotations

import logging
import queue
import threading
import time

import serial

from drivers.lr900p import (
    FrameAssembler,
    ConfigResponse,
    WriteAckResponse,
    build_heartbeat,
    build_config_read,
    build_config_write,
    parse_heartbeat_response,
    parse_config_response,
    parse_write_ack,
    HEARTBEAT_INTERVAL,
    SUBBLOCK_READ,
    SUBBLOCK_WRITE,
)

logger = logging.getLogger(__name__)

_SENTINEL      = object()
_BITS_PER_BYTE = 10  # 1 start + 8 data + 1 stop


class SerialTransport:
    """
    Serial transport for the LR900P telemetry radio.

    Owns the serial port exclusively.  Three daemon threads run while open:

      writer    — single serialized writer; drains _priority_queue first (heartbeats,
                  config frames, ACKs), then _tx_queue (telemetry). All outgoing bytes
                  flow through this one thread — no lock contention, no interleaving.
      heartbeat — enqueues LR900P keepalive frames into _priority_queue every 625 ms.
      reader    — reads all incoming bytes, routes them:
                    • LR900P frames → FrameAssembler → _cfg_queue
                    • all bytes → _cmd_buf for CommandReceiverTask

    Public interface:

      send(frame)              — enqueue a telemetry frame (drops oldest if full)
      send_priority(frame)     — enqueue to priority queue (ACKs, config frames)
      read_available() → bytes — drain _cmd_buf for CommandReceiverTask

      read_config(timeout)     → ConfigResponse | None   (blocking)
      write_config(...)        → WriteAckResponse | None (blocking; modem reboots after)
      is_linked() → bool
      wait_until_open(timeout) → bool
    """

    def __init__(self, port: str, baud: int, write_queue_maxsize: int = 64) -> None:
        self.port = port
        self.baud = baud
        self._secs_per_byte = _BITS_PER_BYTE / baud

        self._serial: serial.Serial | None = None

        # Two outgoing queues — writer drains priority first on every iteration.
        # Priority: heartbeats, ACKs, config frames (small, infrequent, must not be dropped).
        # Normal:   telemetry frames (large, frequent, oldest dropped when full).
        self._priority_queue: queue.Queue[bytes | object] = queue.Queue()
        self._tx_queue:       queue.Queue[bytes | object] = queue.Queue(maxsize=write_queue_maxsize)

        # Incoming bytes for CommandReceiverTask
        self._cmd_buf      = bytearray()
        self._cmd_buf_lock = threading.Lock()

        # LR900P config response queue
        self._cfg_queue: queue.Queue[tuple[int, object]] = queue.Queue()

        # LR900P state
        self._lr_seq        = 0
        self._start_t       = 0.0
        self._linked        = False
        self._config_active = False  # True while read_config/write_config is running

        self._assembler = FrameAssembler(self._on_lr_frame)

        self._writer_thread:    threading.Thread | None = None
        self._reader_thread:    threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._running    = False
        self._open_event = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._open_event.is_set()

    def wait_until_open(self, timeout: float = 10.0) -> bool:
        return self._open_event.wait(timeout=timeout)

    def open(self) -> None:
        self._serial  = serial.Serial(self.port, self.baud, timeout=0.05)
        self._start_t = time.monotonic()
        self._running = True
        self._linked  = False

        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="transport-writer", daemon=True)
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="transport-reader", daemon=True)
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, name="transport-heartbeat", daemon=True)

        self._writer_thread.start()
        self._reader_thread.start()
        self._heartbeat_thread.start()
        self._open_event.set()
        logger.info("SerialTransport: opened %s @ %d baud", self.port, self.baud)

    def close(self) -> None:
        self._running = False
        # Unblock the writer and let it drain before we close the port.
        self._priority_queue.put(_SENTINEL)
        self._tx_queue.put(_SENTINEL)
        if self._writer_thread:
            self._writer_thread.join(timeout=3.0)
        if self._serial and self._serial.is_open:
            # Return radio to transparent bridge mode before closing.
            try:
                self._serial.write(
                    build_heartbeat(self._next_lr_seq(), self._pc_uptime_ms(), link_flag=0x00))
                time.sleep(0.05)
            except Exception:
                pass
            self._serial.close()
        logger.info("SerialTransport: closed")

    # ------------------------------------------------------------------
    # Telemetry TX
    # ------------------------------------------------------------------

    def send(self, frame: bytes) -> None:
        """Enqueue a telemetry frame; drop oldest if full."""
        while True:
            try:
                self._tx_queue.put_nowait(frame)
                return
            except queue.Full:
                try:
                    dropped = self._tx_queue.get_nowait()
                    if isinstance(dropped, bytes):
                        logger.debug("TX queue full — dropped %d-byte frame", len(dropped))
                except queue.Empty:
                    pass

    def send_priority(self, frame: bytes) -> None:
        """Enqueue to the priority queue (ACK frames, config frames). Never dropped."""
        self._priority_queue.put(frame)

    # ------------------------------------------------------------------
    # Command RX
    # ------------------------------------------------------------------

    def read_available(self) -> bytes:
        with self._cmd_buf_lock:
            data = bytes(self._cmd_buf)
            self._cmd_buf.clear()
        return data

    # ------------------------------------------------------------------
    # LR900P config API
    # ------------------------------------------------------------------

    def is_linked(self) -> bool:
        return self._linked

    def read_config(self, timeout: float = 3.0) -> ConfigResponse | None:
        self._flush_cfg_queue()
        self._config_active = True
        try:
            self._priority_queue.put(build_config_read(self._next_lr_seq()))
            return self._wait_cfg(SUBBLOCK_READ, timeout)
        finally:
            self._config_active = False

    def write_config(self, data_rate: int, tx_power: int, channel: int,
                     timeout: float = 3.0) -> WriteAckResponse | None:
        if not 0 <= data_rate <= 2:
            raise ValueError("data_rate must be 0-2")
        if not 0 <= tx_power <= 2:
            raise ValueError("tx_power must be 0-2")
        if not 0 <= channel <= 63:
            raise ValueError("channel must be 0-63")
        self._flush_cfg_queue()
        self._config_active = True
        try:
            self._priority_queue.put(
                build_config_write(self._next_lr_seq(), data_rate, tx_power, channel))
            return self._wait_cfg(SUBBLOCK_WRITE, timeout)
        finally:
            self._config_active = False

    # ------------------------------------------------------------------
    # Internal — LR900P helpers
    # ------------------------------------------------------------------

    def _next_lr_seq(self) -> int:
        s = self._lr_seq
        self._lr_seq = (self._lr_seq + 1) & 0xFF
        return s

    def _pc_uptime_ms(self) -> int:
        return int((time.monotonic() - self._start_t) * 1000) & 0xFFFF

    def _flush_cfg_queue(self) -> None:
        while not self._cfg_queue.empty():
            try:
                self._cfg_queue.get_nowait()
            except queue.Empty:
                break

    def _wait_cfg(self, subblock: int, timeout: float):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            try:
                item = self._cfg_queue.get(timeout=remaining)
                if item[0] == subblock:
                    return item[1]
                self._cfg_queue.put(item)
                time.sleep(0.005)
            except queue.Empty:
                return None

    def _on_lr_frame(self, raw: bytes) -> None:
        if parse_heartbeat_response(raw) is not None:
            self._linked = True
            return
        cfg = parse_config_response(raw)
        if cfg is not None:
            self._cfg_queue.put((SUBBLOCK_READ, cfg))
            return
        ack = parse_write_ack(raw)
        if ack is not None:
            self._cfg_queue.put((SUBBLOCK_WRITE, ack))

    # ------------------------------------------------------------------
    # Threads
    # ------------------------------------------------------------------

    def _writer_loop(self) -> None:
        next_send = time.monotonic()
        while True:
            # Drain priority queue first (non-blocking).
            try:
                item = self._priority_queue.get_nowait()
            except queue.Empty:
                # Nothing urgent — block on normal telemetry queue.
                try:
                    item = self._tx_queue.get(timeout=0.01)
                except queue.Empty:
                    continue

            if item is _SENTINEL:
                break
            if not isinstance(item, bytes):
                continue

            now  = time.monotonic()
            wait = next_send - now
            if wait > 0:
                time.sleep(wait)

            try:
                self._serial.write(item)
                next_send = time.monotonic() + len(item) * self._secs_per_byte
            except serial.SerialException:
                logger.exception("SerialTransport: write error")

    def _heartbeat_loop(self) -> None:
        next_tick = time.monotonic()
        while self._running:
            now = time.monotonic()
            if now >= next_tick:
                link_flag = 0x1E if self._config_active else 0x00
                self._priority_queue.put(
                    build_heartbeat(self._next_lr_seq(), self._pc_uptime_ms(), link_flag))
                next_tick += HEARTBEAT_INTERVAL
            time.sleep(0.01)

    def _reader_loop(self) -> None:
        while self._running:
            try:
                data = self._serial.read(256)
            except Exception:
                if self._running:
                    time.sleep(0.1)
                continue
            if not data:
                continue
            self._assembler.feed(data)
            with self._cmd_buf_lock:
                self._cmd_buf.extend(data)

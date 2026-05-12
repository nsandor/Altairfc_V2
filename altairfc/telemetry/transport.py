from __future__ import annotations

import logging
import queue
import threading
import time

import serial

from drivers.lr900p import (
    FrameAssembler,
    HeartbeatResponse,
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
    SOF,
)

logger = logging.getLogger(__name__)

_SENTINEL    = object()
_BITS_PER_BYTE = 10  # 1 start + 8 data + 1 stop


class SerialTransport:
    """
    Serial transport for the LR900P telemetry radio.

    Owns the serial port exclusively.  Three daemon threads run while open:

      writer    — dequeues outgoing telemetry frames and writes them, baud-paced
      heartbeat — sends LR900P keepalive frames every 625 ms so the modem stays
                  responsive to config requests
      reader    — reads all incoming bytes, routes them:
                    • LR900P frames (SOF 0xEF) → FrameAssembler → internal queues
                    • everything else appended to _cmd_buf for CommandReceiverTask

    Public interface used by tasks:

      send(frame)              — enqueue a telemetry frame (drops oldest if full)
      send_priority(frame)     — write immediately, bypassing the queue
      read_available() → bytes — drain _cmd_buf for CommandReceiverTask

      read_config(timeout)     → ConfigResponse | None   (blocking)
      write_config(...)        → WriteAckResponse | None (blocking; modem reboots after)
      is_linked() → bool       — True once the modem has acknowledged a heartbeat
    """

    def __init__(self, port: str, baud: int, write_queue_maxsize: int = 64) -> None:
        self.port = port
        self.baud = baud
        self._secs_per_byte = _BITS_PER_BYTE / baud

        self._serial: serial.Serial | None = None
        self._write_lock = threading.Lock()

        # Outgoing telemetry queue
        self._tx_queue: queue.Queue[bytes | object] = queue.Queue(maxsize=write_queue_maxsize)

        # Incoming command bytes for CommandReceiverTask
        self._cmd_buf     = bytearray()
        self._cmd_buf_lock = threading.Lock()

        # LR900P config response queues (subblock tagged)
        self._cfg_queue: queue.Queue[tuple[int, object]] = queue.Queue()

        # LR900P state
        self._lr_seq      = 0
        self._start_t     = 0.0
        self._linked      = False
        # True only while a config read/write is in progress — heartbeat uses
        # link_flag=0x1E (config mode) when set, 0x00 (transparent) otherwise.
        self._config_active = False

        self._assembler = FrameAssembler(self._on_lr_frame)

        self._writer_thread:    threading.Thread | None = None
        self._reader_thread:    threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None
        self._running = False
        self._open_event = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @property
    def is_open(self) -> bool:
        return self._open_event.is_set()

    def wait_until_open(self, timeout: float = 10.0) -> bool:
        """Block until open() has been called. Returns True if open within timeout."""
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
        self._tx_queue.put(_SENTINEL)
        if self._writer_thread:
            self._writer_thread.join(timeout=3.0)
        if self._serial and self._serial.is_open:
            # Return radio to transparent bridge mode before closing the port.
            try:
                frame = build_heartbeat(self._next_lr_seq(), self._pc_uptime_ms(), link_flag=0x00)
                self._serial.write(frame)
                time.sleep(0.05)
            except Exception:
                pass
            self._serial.close()
        logger.info("SerialTransport: closed")

    # ------------------------------------------------------------------
    # Telemetry TX (used by TelemetryTask / CommandReceiverTask)
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
        """Write directly to serial, bypassing the TX queue. Used for ACK frames."""
        if self._serial is None or not self._serial.is_open:
            logger.warning("send_priority: port not open, dropping %d-byte frame", len(frame))
            return
        with self._write_lock:
            try:
                self._serial.write(frame)
                time.sleep(len(frame) * self._secs_per_byte)
            except serial.SerialException:
                logger.exception("send_priority: write error")

    # ------------------------------------------------------------------
    # Command RX (used by CommandReceiverTask)
    # ------------------------------------------------------------------

    def read_available(self) -> bytes:
        """Return and clear all buffered non-LR900P bytes received so far."""
        with self._cmd_buf_lock:
            data = bytes(self._cmd_buf)
            self._cmd_buf.clear()
        return data

    # ------------------------------------------------------------------
    # LR900P config API (used by RadioConfigTask)
    # ------------------------------------------------------------------

    def is_linked(self) -> bool:
        return self._linked

    def read_config(self, timeout: float = 3.0) -> ConfigResponse | None:
        self._flush_cfg_queue()
        self._config_active = True
        try:
            with self._write_lock:
                self._serial.write(build_config_read(self._next_lr_seq()))
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
            with self._write_lock:
                self._serial.write(build_config_write(
                    self._next_lr_seq(), data_rate, tx_power, channel))
            return self._wait_cfg(SUBBLOCK_WRITE, timeout)
        finally:
            self._config_active = False

    # ------------------------------------------------------------------
    # Internal — LR900P sequencing & config queue
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
                # Wrong subblock — put it back and yield briefly
                self._cfg_queue.put(item)
                time.sleep(0.005)
            except queue.Empty:
                return None

    # ------------------------------------------------------------------
    # Internal — frame dispatch from reader thread
    # ------------------------------------------------------------------

    def _on_lr_frame(self, raw: bytes) -> None:
        hb = parse_heartbeat_response(raw)
        if hb is not None:
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

            # Partition bytes: LR900P frames start with SOF (0xEF); telemetry/command
            # frames start with SYNC (0xAA). Feed a copy to the LR900P assembler;
            # append everything to _cmd_buf for CommandReceiverTask.
            # The assembler ignores bytes that don't match its framing, so passing
            # all bytes to both consumers is safe — the only cost is a tiny memcpy.
            self._assembler.feed(data)
            with self._cmd_buf_lock:
                self._cmd_buf.extend(data)

    def _heartbeat_loop(self) -> None:
        next_tick = time.monotonic()
        while self._running:
            now = time.monotonic()
            if now >= next_tick:
                # Use 0x1E (config mode) only while a config operation is active;
                # 0x00 (transparent bridge) otherwise so the air link is not blocked.
                link_flag = 0x1E if self._config_active else 0x00
                try:
                    with self._write_lock:
                        self._serial.write(
                            build_heartbeat(self._next_lr_seq(), self._pc_uptime_ms(), link_flag))
                except Exception:
                    pass
                next_tick += HEARTBEAT_INTERVAL
            time.sleep(0.01)

    def _writer_loop(self) -> None:
        assert self._serial is not None
        next_send = time.monotonic()
        while True:
            item = self._tx_queue.get()
            if item is _SENTINEL:
                break
            if not isinstance(item, bytes):
                continue
            now  = time.monotonic()
            wait = next_send - now
            if wait > 0:
                time.sleep(wait)
            try:
                with self._write_lock:
                    self._serial.write(item)
                next_send = time.monotonic() + len(item) * self._secs_per_byte
            except serial.SerialException:
                logger.exception("SerialTransport: write error")

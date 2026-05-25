from __future__ import annotations

import logging
import threading
import time

import serial

logger = logging.getLogger(__name__)

_RECONNECT_DELAY_MAX = 30.0


class BluetoothTransport:
    """
    RFCOMM serial transport for Bluetooth command reception and ACK TX.

    Wraps the RFCOMM device (e.g. /dev/rfcomm0) that the Pi's built-in
    Bluetooth presents after pairing and binding.  Intentionally simpler
    than SerialTransport — no TX queue, no heartbeat, no baud pacing.

    Interface matches the subset of SerialTransport used by CommandReceiverTask:
        read_available() → bytes   — drain RX buffer
        send_priority(frame)       — write ACK frame directly to socket
    """

    def __init__(self, port: str, baud: int = 115200) -> None:
        self.port = port
        self.baud = baud
        self._serial: serial.Serial | None = None
        self._buf      = bytearray()
        self._buf_lock = threading.Lock()
        self._running  = False
        self._reader_thread: threading.Thread | None = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        self._running = True
        self._connect()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="bt-reader", daemon=True)
        self._reader_thread.start()
        logger.info("BluetoothTransport: opened %s", self.port)

    def close(self) -> None:
        self._running = False
        if self._serial and self._serial.is_open:
            self._serial.close()
        logger.info("BluetoothTransport: closed")

    # ------------------------------------------------------------------
    # Interface for CommandReceiverTask
    # ------------------------------------------------------------------

    def read_available(self) -> bytes:
        with self._buf_lock:
            data = bytes(self._buf)
            self._buf.clear()
        return data

    def send_priority(self, frame: bytes) -> None:
        if self._serial is None or not self._serial.is_open:
            return
        try:
            self._serial.write(frame)
        except serial.SerialException as e:
            logger.warning("BluetoothTransport: ACK write failed — %s", e)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _connect(self) -> None:
        delay = 1.0
        while self._running:
            try:
                self._serial = serial.Serial(self.port, self.baud, timeout=0.05)
                logger.info("BluetoothTransport: connected on %s", self.port)
                return
            except serial.SerialException as e:
                logger.warning("BluetoothTransport: waiting for %s (%s) — retry in %.0fs",
                               self.port, e, delay)
                time.sleep(delay)
                delay = min(delay * 2, _RECONNECT_DELAY_MAX)

    def _reader_loop(self) -> None:
        while self._running:
            if self._serial is None or not self._serial.is_open:
                self._connect()
                continue
            try:
                data = self._serial.read(256)
            except serial.SerialException as e:
                msg = str(e)
                # RFCOMM raises this transiently when the remote hasn't sent data yet;
                # it does not mean the connection is gone.
                if "returned no data" in msg:
                    continue
                logger.warning("BluetoothTransport: read error (%s) — reconnecting", e)
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._connect()
                continue
            if data:
                logger.debug("BluetoothTransport: RX %d bytes: %s", len(data), data.hex())
                with self._buf_lock:
                    self._buf.extend(data)

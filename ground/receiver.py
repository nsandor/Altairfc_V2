"""
ALTAIR V2 Ground Station — Telemetry Receiver

Decodes binary telemetry frames from the LR-900p radio over a COM port.
Packet definitions mirror altairfc/telemetry/packets/ exactly.

Usage:
    python receiver.py                        # auto-detect CP210x port
    python receiver.py --port COM5            # specify port explicitly
    python receiver.py --port COM5 --baud 57600 --debug

Requirements:
    pip install pyserial
"""

from __future__ import annotations

import argparse
import binascii
import logging
import struct
import sys
import time
from dataclasses import dataclass
from typing import ClassVar

import serial
import serial.tools.list_ports

# ---------------------------------------------------------------------------
# Logging with colour (mirrors altairfc/core/log_format.py)
# ---------------------------------------------------------------------------

_RESET  = "\033[0m"
_BOLD   = "\033[1m"
_DIM    = "\033[2m"
_GREY   = "\033[38;5;245m"
_CYAN   = "\033[38;5;51m"
_GREEN  = "\033[38;5;82m"
_YELLOW = "\033[38;5;220m"
_ORANGE = "\033[38;5;208m"
_RED    = "\033[38;5;196m"
_PINK   = "\033[38;5;213m"

_LEVEL_STYLES = {
    logging.DEBUG:    (_DIM + _GREY,   _DIM + _GREY),
    logging.INFO:     (_BOLD + _CYAN,  _RESET),
    logging.WARNING:  (_BOLD + _YELLOW, _YELLOW),
    logging.ERROR:    (_BOLD + _ORANGE, _ORANGE),
    logging.CRITICAL: (_BOLD + _RED,   _BOLD + _RED),
}


class _ColorFormatter(logging.Formatter):
    _FMT  = "%(asctime)s  %(levelname)-8s  %(name)s  %(message)s"
    _DATE = "%Y-%m-%dT%H:%M:%S"

    def __init__(self, use_color: bool) -> None:
        super().__init__(fmt=self._FMT, datefmt=self._DATE)
        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        if not self.use_color:
            return super().format(record)
        level_color, msg_color = _LEVEL_STYLES.get(record.levelno, (_RESET, _RESET))
        record.levelname = f"{level_color}{record.levelname:<8}{_RESET}"
        record.name      = f"{_DIM}{_PINK}{record.name}{_RESET}"
        formatted        = super().format(record)
        plain_msg        = record.getMessage()
        formatted        = formatted.replace(plain_msg, f"{msg_color}{plain_msg}{_RESET}", 1)
        formatted        = f"{_DIM}{_GREY}{formatted[:19]}{_RESET}{formatted[19:]}"
        return formatted


def _setup_logging(level: str) -> None:
    use_color = hasattr(sys.stdout, "isatty") and sys.stdout.isatty()
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(_ColorFormatter(use_color))
    root = logging.getLogger()
    root.handlers.clear()
    root.addHandler(handler)
    root.setLevel(level)


logger = logging.getLogger("ground.receiver")

# ---------------------------------------------------------------------------
# Packet definitions  (must stay in sync with altairfc/telemetry/packets/)
# ---------------------------------------------------------------------------
# Wire format per field:  'f' = float32 little-endian (4 bytes)
# Add/remove fields here to match the flight computer side.

@dataclass
class HeartbeatPacket:
    """Packet ID 0x00 — system heartbeat."""
    PACKET_ID:    ClassVar[int]          = 0x00
    STRUCT_FMT:   ClassVar[struct.Struct] = struct.Struct("<ddffffffffff")
    FIELD_NAMES:  ClassVar[tuple]        = (
        "time_unix", "uptime_s",
        "cpu_load_pct", "mem_used_pct", "tasks_running",
        "pixhawk_connected", "vesc_connected", "power_connected", "photodiode_connected",
        "pps_synced", "pps_rms_us",
    )
    UNITS: ClassVar[tuple] = ("s", "s", "%", "%", "count", "bool", "bool", "bool", "bool", "bool", "us")

    time_unix:            float = 0.0
    uptime_s:             float = 0.0
    cpu_load_pct:         float = 0.0
    mem_used_pct:         float = 0.0
    tasks_running:        float = 0.0
    pixhawk_connected:    float = 0.0
    vesc_connected:       float = 0.0
    power_connected:      float = 0.0
    photodiode_connected: float = 0.0
    pps_synced:           float = 0.0
    pps_rms_us:           float = 0.0


@dataclass
class AttitudePacket:
    """Packet ID 0x01 — MAVLink ATTITUDE from Pixhawk 6X mini."""
    PACKET_ID:    ClassVar[int]          = 0x01
    STRUCT_FMT:   ClassVar[struct.Struct] = struct.Struct("<ffffff")
    FIELD_NAMES:  ClassVar[tuple]        = ("roll", "pitch", "yaw",
                                             "rollspeed", "pitchspeed", "yawspeed")
    UNITS:        ClassVar[tuple]        = ("rad", "rad", "rad",
                                             "rad/s", "rad/s", "rad/s")

    roll:       float = 0.0
    pitch:      float = 0.0
    yaw:        float = 0.0
    rollspeed:  float = 0.0
    pitchspeed: float = 0.0
    yawspeed:   float = 0.0


@dataclass
class PowerPacket:
    """Packet ID 0x02 — INA3221 three-rail power monitor."""
    PACKET_ID:    ClassVar[int]          = 0x02
    STRUCT_FMT:   ClassVar[struct.Struct] = struct.Struct("<ffffff")
    FIELD_NAMES:  ClassVar[tuple]        = ("voltage_24v", "current_24v", "voltage_12v", "current_12v", "voltage_5v", "current_5v")
    UNITS:        ClassVar[tuple]        = ("V", "A", "V", "A", "V", "A")

    voltage_24v: float = 0.0
    current_24v: float = 0.0
    voltage_12v: float = 0.0
    current_12v: float = 0.0
    voltage_5v:  float = 0.0
    current_5v:  float = 0.0


@dataclass
class VescPacket:
    """Packet ID 0x03 — VESC motor controller."""
    PACKET_ID:    ClassVar[int]          = 0x03
    STRUCT_FMT:   ClassVar[struct.Struct] = struct.Struct("<fffff")
    FIELD_NAMES:  ClassVar[tuple]        = ("rpm", "duty_cycle", "motor_current",
                                             "input_voltage", "temperature_mos")
    UNITS:        ClassVar[tuple]        = ("rpm", "%", "A", "V", "degC")

    rpm:             float = 0.0
    duty_cycle:      float = 0.0
    motor_current:   float = 0.0
    input_voltage:   float = 0.0
    temperature_mos: float = 0.0


@dataclass
class PhotodiodePacket:
    """Packet ID 0x04 — fixed UVIC PDRO test-campaign measurements."""
    PACKET_ID:    ClassVar[int]          = 0x04
    STRUCT_FMT:   ClassVar[struct.Struct] = struct.Struct("<ffffff")
    FIELD_NAMES:  ClassVar[tuple]        = (
        "sergeant_tia_low_gain",
        "soldier_tia_low_gain",
        "sergeant_board_temperature",
        "soldier_board_temperature",
        "sergeant_photodiode_temperature",
        "soldier_photodiode_temperature",
    )
    UNITS:        ClassVar[tuple]        = ("V", "V", "C", "C", "C", "C")

    sergeant_tia_low_gain: float = 0.0
    soldier_tia_low_gain: float = 0.0
    sergeant_board_temperature: float = 0.0
    soldier_board_temperature: float = 0.0
    sergeant_photodiode_temperature: float = 0.0
    soldier_photodiode_temperature: float = 0.0


@dataclass
class FlightSettingsPacket:
    """Packet ID 0x09 — Active flight configuration snapshot."""
    PACKET_ID:   ClassVar[int]           = 0x09
    STRUCT_FMT:  ClassVar[struct.Struct] = struct.Struct("<ffffffffffffffff")  # 16 × float32
    FIELD_NAMES: ClassVar[tuple]         = (
        "termination_altitude_m", "burst_altitude_m", "burst_altitude_uncertainty_m",
        "ascent_detect_window_s", "ascent_detect_gain_m", "apogee_fraction",
        "landing_fraction", "recovery_stationary_s", "termination_confirm_drop_m",
        "termination_confirm_window_s",
        "rw_kp", "rw_kd", "rw_max_rpm",
        "mm_kp", "mm_kd", "mm_max_current",
    )
    UNITS: ClassVar[tuple] = (
        "m", "m", "m", "s", "m", "fraction", "fraction", "s", "m", "s",
        "RPM/rad/s", "RPM/rad/s2", "RPM",
        "A/rad/s", "A/rad/s2", "mA",
    )

    termination_altitude_m:       float = 0.0
    burst_altitude_m:             float = 0.0
    burst_altitude_uncertainty_m: float = 0.0
    ascent_detect_window_s:       float = 0.0
    ascent_detect_gain_m:         float = 0.0
    apogee_fraction:              float = 0.0
    landing_fraction:             float = 0.0
    recovery_stationary_s:        float = 0.0
    termination_confirm_drop_m:   float = 0.0
    termination_confirm_window_s: float = 0.0
    rw_kp:                        float = 0.0
    rw_kd:                        float = 0.0
    rw_max_rpm:                   float = 0.0
    mm_kp:                        float = 0.0
    mm_kd:                        float = 0.0
    mm_max_current:               float = 0.0


@dataclass
class LocalGpsPacket:
    """Packet ID 0x05 — onboard MAX-M10M GPS module."""
    PACKET_ID:    ClassVar[int]          = 0x05
    STRUCT_FMT:   ClassVar[struct.Struct] = struct.Struct("<Bfffff BB")
    FIELD_NAMES:  ClassVar[tuple]        = ("active", "lat", "lon", "alt_msl", "speed_ms", "heading_deg", "fix_type", "num_sv")
    UNITS:        ClassVar[tuple]        = ("", "deg", "deg", "m", "m/s", "deg", "", "")

    active:      int   = 0
    lat:         float = 0.0
    lon:         float = 0.0
    alt_msl:     float = 0.0
    speed_ms:    float = 0.0
    heading_deg: float = 0.0
    fix_type:    int   = 0
    num_sv:      int   = 0


@dataclass
class PointingPacket:
    """Packet ID 0x0A — pointing state (heading error, az error, RW saturation, GPS2 heading)."""
    PACKET_ID:    ClassVar[int]          = 0x0A
    STRUCT_FMT:   ClassVar[struct.Struct] = struct.Struct("<ffff")
    FIELD_NAMES:  ClassVar[tuple]        = ("heading_error", "az_error", "rw_saturated", "heading")
    UNITS:        ClassVar[tuple]        = ("deg", "rad", "", "deg")

    heading_error: float = 0.0
    az_error:      float = 0.0
    rw_saturated:  float = 0.0
    heading:       float = 0.0


# Registry: packet_id -> class
_PACKET_REGISTRY: dict[int, type] = {
    cls.PACKET_ID: cls
    for cls in (HeartbeatPacket, AttitudePacket, PowerPacket, VescPacket, PhotodiodePacket, FlightSettingsPacket, LocalGpsPacket, PointingPacket)
}

# ---------------------------------------------------------------------------
# Settings update command (GS→FC)
# Field IDs must match SETTING_KEYS in altairfc/telemetry/commands/update_setting.py
# ---------------------------------------------------------------------------
SETTING_KEYS: tuple[str, ...] = (
    "termination_altitude_m",        # 0
    "burst_altitude_m",              # 1
    "burst_altitude_uncertainty_m",  # 2
    "ascent_detect_window_s",        # 3
    "ascent_detect_gain_m",          # 4
    "apogee_fraction",               # 5
    "landing_fraction",              # 6
    "recovery_stationary_s",         # 7
    "termination_confirm_drop_m",    # 8
    "termination_confirm_window_s",  # 9
    "rw_kp",                         # 10
    "rw_kd",                         # 11
    "rw_max_rpm",                    # 12
    "mm_kp",                         # 13
    "mm_kd",                         # 14
    "mm_max_current",                # 15
)

_UPDATE_SETTING_CMD_ID = 0xC3
_SETTING_PAYLOAD       = struct.Struct("<Bf")   # uint8 field_id + float32 value
_cmd_seq: int          = 0


def send_settings_update_command(ser: serial.Serial, field_id: int, value: float) -> None:
    """
    Send an UpdateSettingCommand frame to the flight computer.

    field_id indexes into SETTING_KEYS. The FC will write the new value to
    the corresponding DataStore key and return an ACK.

    Example — lower cutdown altitude to 20 km:
        send_settings_update_command(ser, 0, 20000.0)
    """
    global _cmd_seq
    if not (0 <= field_id < len(SETTING_KEYS)):
        raise ValueError(f"field_id {field_id} out of range (0–{len(SETTING_KEYS) - 1})")
    payload    = _SETTING_PAYLOAD.pack(field_id, value)
    timestamp  = time.time()
    header     = _HEADER.pack(SYNC_BYTE, _UPDATE_SETTING_CMD_ID, _cmd_seq & 0xFF, timestamp, len(payload))
    crc        = binascii.crc_hqx(header[1:] + payload, 0xFFFF)
    frame      = header + payload + _CRC.pack(crc)
    ser.write(frame)
    logger.info(
        "Sent UpdateSetting: field_id=%d (%s) = %g",
        field_id, SETTING_KEYS[field_id], value,
    )
    _cmd_seq = (_cmd_seq + 1) & 0xFF

# ---------------------------------------------------------------------------
# Frame constants  (must match altairfc/telemetry/serializer.py)
# ---------------------------------------------------------------------------
SYNC_BYTE     = 0xAA
_HEADER       = struct.Struct("<BBBdH")   # sync, pkt_id, seq, timestamp, length
_CRC          = struct.Struct("<H")
HEADER_SIZE   = _HEADER.size              # 13 bytes
CRC_SIZE      = _CRC.size                 # 2 bytes
MIN_FRAME     = HEADER_SIZE + CRC_SIZE    # 15 bytes


# ---------------------------------------------------------------------------
# Port auto-detection (CP210x = Silicon Labs, used by LR-900p)
# ---------------------------------------------------------------------------
_CP210X_VID = 0x10C4
_CP210X_PID = 0xEA60


def find_lr900p_port() -> str | None:
    matches = [
        p for p in serial.tools.list_ports.comports()
        if p.vid == _CP210X_VID and p.pid == _CP210X_PID
    ]
    if not matches:
        return None
    if len(matches) > 1:
        logger.warning(
            "Multiple CP210x devices found %s — using %s. "
            "Pass --port explicitly to suppress this.",
            [p.device for p in matches], matches[0].device,
        )
    logger.info("Auto-detected LR-900p on %s (%s)", matches[0].device, matches[0].description)
    return matches[0].device


# ---------------------------------------------------------------------------
# Decoder
# ---------------------------------------------------------------------------

def _verify_crc(raw: bytes, payload_len: int) -> bool:
    received = _CRC.unpack_from(raw, HEADER_SIZE + payload_len)[0]
    computed = binascii.crc_hqx(raw[1 : HEADER_SIZE + payload_len], 0xFFFF)
    return received == computed


def decode_frame(raw: bytes) -> tuple[object, int, float] | None:
    """
    Decode one complete frame.

    Returns (packet_instance, seq, timestamp) on success, None on failure.
    """
    if len(raw) < MIN_FRAME:
        return None

    sync, pkt_id, seq, timestamp, length = _HEADER.unpack_from(raw, 0)

    if sync != SYNC_BYTE:
        return None

    if len(raw) < HEADER_SIZE + length + CRC_SIZE:
        return None

    if not _verify_crc(raw, length):
        logger.warning("CRC mismatch — frame dropped (PKT_ID=0x%02X SEQ=%d)", pkt_id, seq)
        return None

    pkt_class = _PACKET_REGISTRY.get(pkt_id)
    if pkt_class is None:
        logger.warning("Unknown packet ID 0x%02X — skipping", pkt_id)
        return None

    payload = raw[HEADER_SIZE : HEADER_SIZE + length]
    if len(payload) != pkt_class.STRUCT_FMT.size:
        logger.warning(
            "Payload size mismatch for 0x%02X: expected %d got %d",
            pkt_id, pkt_class.STRUCT_FMT.size, len(payload),
        )
        return None

    values = pkt_class.STRUCT_FMT.unpack(payload)
    packet = pkt_class(**dict(zip(pkt_class.FIELD_NAMES, values)))
    return packet, seq, timestamp


def _print_packet(packet: object, seq: int, timestamp: float) -> None:
    cls = type(packet)
    name = cls.__name__.replace("Packet", "")

    logger.info(
        "[%s] seq=%d  ts=%.3f",
        name, seq, timestamp,
    )
    for field_name, unit in zip(cls.FIELD_NAMES, cls.UNITS):
        value = getattr(packet, field_name)
        logger.info("    %-18s %10.4f  %s", field_name, value, unit)


# ---------------------------------------------------------------------------
# Serial reader with SYNC-byte framing
# ---------------------------------------------------------------------------

class FrameReader:
    """
    Reads bytes from serial, scans for SYNC (0xAA), then buffers a complete
    frame before handing it to decode_frame().
    """

    def __init__(self, port: serial.Serial) -> None:
        self._port = port
        self._buf = bytearray()
        self._seq_prev: dict[int, int] = {}
        self._frames_received = 0
        self._frames_dropped  = 0

    def run(self) -> None:
        logger.info("Listening for telemetry frames on %s...", self._port.name)
        while True:
            chunk = self._port.read(256)
            if not chunk:
                continue
            self._buf.extend(chunk)
            self._process_buffer()

    def _process_buffer(self) -> None:
        while len(self._buf) >= MIN_FRAME:
            # Scan forward to SYNC byte
            sync_pos = self._buf.find(SYNC_BYTE)
            if sync_pos == -1:
                self._buf.clear()
                return
            if sync_pos > 0:
                logger.debug("Skipping %d non-SYNC bytes", sync_pos)
                del self._buf[:sync_pos]

            # Need at least a full header to know frame length
            if len(self._buf) < HEADER_SIZE:
                return

            _, _, _, _, length = _HEADER.unpack_from(self._buf, 0)
            frame_size = HEADER_SIZE + length + CRC_SIZE

            if len(self._buf) < frame_size:
                return  # wait for more bytes

            frame = bytes(self._buf[:frame_size])
            del self._buf[:frame_size]

            result = decode_frame(frame)
            if result is None:
                self._frames_dropped += 1
                # Discard only the SYNC byte and retry from next position
                continue

            packet, seq, timestamp = result
            self._frames_received += 1
            self._check_seq(type(packet).PACKET_ID, seq)
            _print_packet(packet, seq, timestamp)

    def _check_seq(self, pkt_id: int, seq: int) -> None:
        prev = self._seq_prev.get(pkt_id)
        if prev is not None:
            expected = (prev + 1) & 0xFF
            if seq != expected:
                dropped = (seq - expected) & 0xFF
                logger.warning(
                    "Sequence gap on PKT_ID=0x%02X: expected %d got %d (%d frame(s) dropped)",
                    pkt_id, expected, seq, dropped,
                )
        self._seq_prev[pkt_id] = seq


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="ALTAIR V2 ground station telemetry receiver")
    parser.add_argument("--port",  default="auto", help="COM port (default: auto-detect CP210x)")
    parser.add_argument("--baud",  type=int, default=57600, help="Baud rate (default: 57600)")
    parser.add_argument("--debug", action="store_true",     help="Enable DEBUG logging")
    args = parser.parse_args()

    _setup_logging("DEBUG" if args.debug else "INFO")

    port = args.port
    if port.lower() == "auto":
        port = find_lr900p_port()
        if port is None:
            logger.critical(
                "No CP210x device found. Connect the LR-900p or pass --port COM<N> explicitly."
            )
            sys.exit(1)

    try:
        ser = serial.Serial(port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        logger.critical("Could not open %s: %s", port, e)
        sys.exit(1)

    logger.info("Opened %s @ %d baud", port, args.baud)

    try:
        FrameReader(ser).run()
    except KeyboardInterrupt:
        logger.info("Interrupted — closing port")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

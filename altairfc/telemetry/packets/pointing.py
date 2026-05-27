from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x0A)
@dataclass
class PointingPacket:
    """
    Pointing state telemetry.
    Packet ID: 0x0A
    Payload size: 4 * 4 = 16 bytes (all float32)

    DataStore keys:
        pointing.heading_error   — signed heading error to 0° target (deg, -180..+180)
        pointing.az_error        — azimuth error in body frame from quaternion (rad)
        pointing.rw_saturated    — 1.0 if RW is saturated, 0.0 otherwise
        mavlink.heading          — GPS2 dual-antenna heading (deg, 0..360)
    """

    TX_RATE_HZ: ClassVar[float] = 2.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "heading_error":  "pointing.heading_error",
        "az_error":       "pointing.az_error",
        "rw_saturated":   "pointing.rw_saturated",
        "heading":        "mavlink.heading",
    }

    heading_error: float = field(default=0.0, metadata=FieldMeta("f", "Heading error to 0 deg target", "deg", min_val=-180.0, max_val=180.0).as_metadata())
    az_error:      float = field(default=0.0, metadata=FieldMeta("f", "Azimuth error in body frame",   "rad", min_val=-3.1416, max_val=3.1416).as_metadata())
    rw_saturated:  float = field(default=0.0, metadata=FieldMeta("f", "RW saturation flag",            "",    min_val=0.0,    max_val=1.0   ).as_metadata())
    heading:       float = field(default=0.0, metadata=FieldMeta("f", "GPS2 dual-antenna heading",     "deg", min_val=0.0,    max_val=360.0 ).as_metadata())

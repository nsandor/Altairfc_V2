from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x04)
@dataclass
class PhotodiodePacket:
    """Fixed UVIC PDRO test-campaign measurements."""

    TX_RATE_HZ: ClassVar[float] = 2.0

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "sergeant_tia_low_gain": "photodiode.sergeant.tia_low_gain",
        "soldier_tia_low_gain": "photodiode.soldier.tia_low_gain",
        "sergeant_board_temperature": "photodiode.sergeant.board_temperature",
        "soldier_board_temperature": "photodiode.soldier.board_temperature",
        "sergeant_photodiode_temperature": (
            "photodiode.sergeant.photodiode_temperature"
        ),
        "soldier_photodiode_temperature": (
            "photodiode.soldier.photodiode_temperature"
        ),
    }

    sergeant_tia_low_gain: float = field(
        default=0.0,
        metadata=FieldMeta("f", "Sergeant low-gain TIA", "V").as_metadata(),
    )
    soldier_tia_low_gain: float = field(
        default=0.0,
        metadata=FieldMeta("f", "Soldier low-gain TIA", "V").as_metadata(),
    )
    sergeant_board_temperature: float = field(
        default=0.0,
        metadata=FieldMeta("f", "Sergeant board temperature", "C").as_metadata(),
    )
    soldier_board_temperature: float = field(
        default=0.0,
        metadata=FieldMeta("f", "Soldier board temperature", "C").as_metadata(),
    )
    sergeant_photodiode_temperature: float = field(
        default=0.0,
        metadata=FieldMeta(
            "f", "Sergeant photodiode temperature", "C"
        ).as_metadata(),
    )
    soldier_photodiode_temperature: float = field(
        default=0.0,
        metadata=FieldMeta(
            "f", "Soldier photodiode temperature", "C"
        ).as_metadata(),
    )

from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.registry import FieldMeta, packet_registry


@packet_registry.register(packet_id=0x0C)
@dataclass
class RadioConfigPacket:
    """
    Current LR900P radio configuration — read back from the modem after startup
    and after each write_config().

    Packet ID: 0x0C
    Payload size: 3 bytes (all uint8)

    DataStore keys written by RadioConfigTask:
        radio.data_rate  — 0=Low, 1=Mid, 2=High
        radio.tx_power   — 0=Low, 1=Mid, 2=High
        radio.channel    — 0-63
    """

    TX_RATE_HZ: ClassVar[float] = 0.2

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "data_rate": "radio.data_rate",
        "tx_power":  "radio.tx_power",
        "channel":   "radio.channel",
    }

    _RC = "Radio Config"

    data_rate: int = field(default=0, metadata=FieldMeta("B", "Data rate (0=Low,1=Mid,2=High)", "",    group=_RC, min_val=0, max_val=2).as_metadata())
    tx_power:  int = field(default=0, metadata=FieldMeta("B", "TX power (0=Low,1=Mid,2=High)",  "",    group=_RC, min_val=0, max_val=2).as_metadata())
    channel:   int = field(default=0, metadata=FieldMeta("B", "RF channel (0-63)",              "",    group=_RC, min_val=0, max_val=63).as_metadata())

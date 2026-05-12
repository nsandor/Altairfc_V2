from __future__ import annotations

from dataclasses import dataclass, field
from typing import ClassVar

from telemetry.command_registry import command_registry
from telemetry.registry import FieldMeta


@command_registry.register(packet_id=0xC5)
@dataclass
class RadioConfigCommandPacket:
    """
    RADIO_CONFIG command sent from GS to FC.
    Command ID: 0xC5
    Payload: 3 bytes (3 × uint8)

    Carries the desired LR900P radio configuration.

    Effect: sets DataStore keys:
        "command.radio_data_rate"  — 0=Low, 1=Mid, 2=High
        "command.radio_tx_power"   — 0=Low, 1=Mid, 2=High
        "command.radio_channel"    — 0-63
    RadioConfigTask detects these keys, calls write_config(), then re-reads
    the modem to confirm and updates radio.* keys.
    """

    DATASTORE_KEYS: ClassVar[dict[str, str]] = {
        "data_rate": "command.radio_data_rate",
        "tx_power":  "command.radio_tx_power",
        "channel":   "command.radio_channel",
    }

    data_rate: int = field(default=0, metadata=FieldMeta("B", "Data rate (0=Low,1=Mid,2=High)", "", min_val=0, max_val=2).as_metadata())
    tx_power:  int = field(default=0, metadata=FieldMeta("B", "TX power (0=Low,1=Mid,2=High)",  "", min_val=0, max_val=2).as_metadata())
    channel:   int = field(default=0, metadata=FieldMeta("B", "RF channel (0-63)",              "", min_val=0, max_val=63).as_metadata())

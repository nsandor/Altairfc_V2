from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from ground.receiver import PhotodiodePacket as GroundPhotodiodePacket
from ground.receiver import decode_frame
from telemetry.packets.photodiode import PhotodiodePacket
from telemetry.serializer import PacketSerializer


def test_ground_receiver_decodes_fixed_photodiode_packet():
    frame = PacketSerializer().pack(
        PhotodiodePacket(
            sergeant_tia_low_gain=1.0,
            soldier_tia_low_gain=2.0,
            sergeant_board_temperature=3.0,
            soldier_board_temperature=4.0,
            sergeant_photodiode_temperature=5.0,
            soldier_photodiode_temperature=6.0,
        ),
        seq=42,
    )

    decoded = decode_frame(frame)

    assert decoded is not None
    packet, sequence, _ = decoded
    assert isinstance(packet, GroundPhotodiodePacket)
    assert sequence == 42
    assert packet.sergeant_tia_low_gain == 1.0
    assert packet.soldier_tia_low_gain == 2.0
    assert packet.sergeant_board_temperature == 3.0
    assert packet.soldier_board_temperature == 4.0
    assert packet.sergeant_photodiode_temperature == 5.0
    assert packet.soldier_photodiode_temperature == 6.0

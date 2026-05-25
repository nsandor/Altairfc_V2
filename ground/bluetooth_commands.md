# ALTAIR V2 — Bluetooth Command Interface

Bluetooth provides an alternative command channel for use when the LR900P radio is configured
for TX-only mode (e.g. continuous telemetry downlink with no uplink). When the radio is in
full-duplex mode, commands work normally over radio and Bluetooth is redundant but harmless.

The Pi 4B's built-in Bluetooth is configured as an RFCOMM serial slave. The GS Windows laptop
pairs once and sends the same binary command frames used on the radio link. The FC replies with
ACK frames on **all active transports** — both Bluetooth and radio — so receipt can be confirmed
on whichever channel the GS is monitoring.

---

## 1. One-time Pi setup

Run these commands once on the Pi (or add them to the flight service pre-start):

```bash
# Ensure bluetoothd is running
sudo systemctl enable --now bluetooth

# Make discoverable and pair interactively
bluetoothctl
  > power on
  > discoverable on
  > agent on
  > default-agent
  # — pair from the Windows laptop now —
  # accept the pairing PIN when prompted
  > trust <LAPTOP_MAC>
  > quit

# Bind RFCOMM channel 1 to the paired laptop address
# Replace XX:XX:XX:XX:XX:XX with the laptop Bluetooth MAC
sudo rfcomm bind 0 XX:XX:XX:XX:XX:XX 1

# /dev/rfcomm0 now exists. The flight service opens it on startup if
# [bluetooth] enabled = true in config/settings.toml
```

To make the bind persist across reboots, add a systemd override or `/etc/rc.local` entry:

```bash
rfcomm bind 0 XX:XX:XX:XX:XX:XX 1
```

---

## 2. Windows laptop setup

1. Open **Settings → Bluetooth & devices → Add device** and pair with "ALTAIR-V2" (or the Pi's hostname).
2. Open **Device Manager → Ports (COM & LPT)** — note the COM port number assigned to the outgoing RFCOMM service (e.g. `COM5`).
3. Open the serial port in any terminal or the GS software at **115200 baud, 8N1, no flow control**.
4. The FC does not send telemetry over Bluetooth — only ACK frames are transmitted back.

---

## 3. Frame format

All frames (both GS→FC commands and FC→GS ACKs) use the standard ALTAIR binary frame:

```
Offset  Size  Type    Field
──────  ────  ──────  ──────────────────────────────────────────────────
0       1     uint8   SYNC = 0xAA
1       1     uint8   PKT_ID  (command ID or ACK ID)
2       1     uint8   SEQ     (per-frame counter, 0–255, wraps)
3       8     float64 TIMESTAMP (Unix time, little-endian)
11      2     uint16  LEN     (payload length in bytes, little-endian)
13      LEN   —       PAYLOAD (little-endian fields)
13+LEN  2     uint16  CRC16   (CRC-16/CCITT over bytes 1..13+LEN-1, seed 0xFFFF, little-endian)
```

Header is always 13 bytes. Minimum frame size (zero-payload) is 15 bytes.

CRC covers everything between SYNC and CRC (exclusive): `bytes[1 : 13 + LEN]`.
Python: `binascii.crc_hqx(frame[1:13+LEN], 0xFFFF)`.

---

## 4. Command reference

### 0xC0 — ARM
Arms the flight computer. FlightStageTask transitions to STAGE_ARMED on next cycle.

| Offset | Size | Type   | Field     | Value |
|--------|------|--------|-----------|-------|
| 0      | 1    | uint8  | arm_state | 1     |

Total payload: **1 byte**.

### 0xC1 — LAUNCH_OK
Authorises launch. Only accepted when FC is in STAGE_ARMED; otherwise ACK status = 1 (rejected).

| Offset | Size | Type   | Field   | Value |
|--------|------|--------|---------|-------|
| 0      | 1    | uint8  | confirm | 1     |

Total payload: **1 byte**.

### 0xC2 — PING
Tests round-trip link. The FC sends an ACK with `cmd_seq` echoing the frame SEQ field.
No DataStore side effect.

| Offset | Size | Type   | Field | Value       |
|--------|------|--------|-------|-------------|
| 0      | 1    | uint8  | token | any 0–255   |

Total payload: **1 byte**.

### 0xC3 — UPDATE_SETTING
Updates one flight parameter by index without a reboot.

| Offset | Size | Type    | Field    | Notes                   |
|--------|------|---------|----------|-------------------------|
| 0      | 1    | uint8   | field_id | see table below         |
| 1      | 4    | float32 | value    | little-endian           |

Total payload: **5 bytes**.

**field_id table:**

| ID | DataStore key                          | Units   |
|----|----------------------------------------|---------|
| 0  | settings.termination_altitude_m        | m       |
| 1  | settings.burst_altitude_m              | m       |
| 2  | settings.burst_altitude_uncertainty_m  | m       |
| 3  | settings.ascent_detect_window_s        | s       |
| 4  | settings.ascent_detect_gain_m          | m       |
| 5  | settings.apogee_fraction               | 0–1     |
| 6  | settings.landing_fraction              | 0–1     |
| 7  | settings.recovery_stationary_s         | s       |
| 8  | settings.termination_confirm_drop_m    | m       |
| 9  | settings.termination_confirm_window_s  | s       |
| 10 | settings.rw_kp                         | —       |
| 11 | settings.rw_kd                         | —       |
| 12 | settings.rw_max_rpm                    | RPM     |
| 13 | settings.mm_kp                         | —       |
| 14 | settings.mm_kd                         | —       |
| 15 | settings.mm_max_current                | mA      |
| 16 | settings.pointing_activate_altitude_m  | m       |
| 17 | settings.pointing_duration_min         | min     |

### 0xC4 — GS_GPS
Sends the ground station GPS position so the FC can compute pointing vector.
The FC writes these to DataStore; if `settings.gs_use_hardcoded = 0`, these override the
hardcoded coordinates from settings.toml.

| Offset | Size | Type    | Field | Notes                         |
|--------|------|---------|-------|-------------------------------|
| 0      | 4    | float32 | lat   | degrees, +N, little-endian    |
| 4      | 4    | float32 | lon   | degrees, +E, little-endian    |
| 8      | 4    | float32 | alt   | metres MSL, little-endian     |

Total payload: **12 bytes**.

### 0xC5 — RADIO_CONFIG
Requests a radio parameter change. Only processed before STAGE_ARMED; rejected afterwards.
Can be sent over Bluetooth or radio — `RadioConfigTask` reads the resulting DataStore keys
regardless of which transport delivered the command. Requires `config_enabled = true` in
`[radio_config]` of settings.toml; otherwise the keys are written but RadioConfigTask ignores them.

| Offset | Size | Type   | Field     | Values              |
|--------|------|--------|-----------|---------------------|
| 0      | 1    | uint8  | data_rate | 0=Low 1=Mid 2=High  |
| 1      | 1    | uint8  | tx_power  | 0=Low 1=Mid 2=High  |
| 2      | 1    | uint8  | channel   | 0–63                |

Total payload: **3 bytes**.

---

## 5. ACK frame (FC → GS)

Packet ID: **0xA0**

| Offset | Size | Type   | Field   | Notes                            |
|--------|------|--------|---------|----------------------------------|
| 0      | 1    | uint8  | cmd_id  | PKT_ID of the command being acked|
| 1      | 1    | uint8  | cmd_seq | SEQ echoed from command header   |
| 2      | 1    | uint8  | status  | 0 = accepted, 1 = rejected       |

Total payload: **3 bytes**. Full frame is 18 bytes.

The FC sends one ACK for every valid command frame received, regardless of whether the command
was accepted or rejected. If no ACK arrives within ~2 s, retransmit the command.

---

## 6. Python helper snippet

```python
import binascii
import struct
import time

SYNC      = 0xAA
HDR       = struct.Struct("<BBBdH")
CRC_STRUCT = struct.Struct("<H")

def build_frame(pkt_id: int, payload: bytes, seq: int = 0) -> bytes:
    header = HDR.pack(SYNC, pkt_id & 0xFF, seq & 0xFF, time.time(), len(payload))
    crc_data = header[1:] + payload
    crc = binascii.crc_hqx(crc_data, 0xFFFF)
    return header + payload + CRC_STRUCT.pack(crc)

# Examples
arm_frame         = build_frame(0xC0, struct.pack("B", 1))
launch_ok_frame   = build_frame(0xC1, struct.pack("B", 1))
ping_frame        = build_frame(0xC2, struct.pack("B", 42))
update_setting    = build_frame(0xC3, struct.pack("<Bf", 0, 25000.0))  # field 0 = termination alt
gs_gps_frame      = build_frame(0xC4, struct.pack("<fff", 45.494, -73.552, 55.0))
```

---

## 7. Operational notes

- Bluetooth and radio are **independent channels**. Commands sent over Bluetooth do not appear
  in the radio log and vice versa. The FC processes both simultaneously.
- ACKs are sent on **all active transports**, regardless of which channel the command arrived on.
  A command sent via Bluetooth generates both a Bluetooth ACK and a radio ACK (if the radio
  transport is open). This lets the GS operator confirm receipt on whichever channel they are
  monitoring.
- The FC tracks `system.last_gs_contact_t` on every ACK sent, from both channels. The radio
  watchdog in `RadioConfigTask` uses this key to detect loss of contact.
- If `/dev/rfcomm0` is not bound (Pi side) or the COM port is not open (Windows side), the FC
  Bluetooth task will retry the connection automatically with exponential backoff up to 30 s.
- To disable Bluetooth entirely: set `[bluetooth] enabled = false` in `config/settings.toml`.

# Range Rover BLE — Home Assistant Custom Integration

[![hacs_badge](https://img.shields.io/badge/HACS-Custom-41BDF5.svg)](https://github.com/hacs/integration)

A [Home Assistant](https://www.home-assistant.io/) custom integration for monitoring **Range Rover P550e PHEV** battery and vehicle data via a Bluetooth Low Energy (BLE) ELM327 OBD-II adapter.

When your Range Rover pulls into the garage, this integration automatically connects over BLE and reads live battery diagnostics — state of charge, state of health, voltage, current, temperature, and more — making them available as native Home Assistant sensors.

> **Status: Early development.** PIDs are based on JLR BECM research from I-Pace community data and need validation on the actual P550e vehicle. See [Discovery](#discovery-approach) below.

---

## Features

- **HV Battery State of Charge** — real-time battery percentage (BECM DID 0x4910)
- **State of Health** — long-term battery degradation tracking (DID 0x4918)
- **Battery voltage, current, temperature** — pack-level monitoring
- **Cell voltage min/max** — cell-level health indicators
- **12V battery voltage** — auxiliary battery monitoring
- **Standard OBD data** — speed, ambient temperature
- Communicates over **Bluetooth Low Energy** — no Wi-Fi or cloud dependency
- Works with **ESPHome Bluetooth proxies** for remote garage setups
- Self-contained — no external Python packages beyond `bleak` (bundled with HA)

---

## Requirements

### Hardware

| Item | Details |
|---|---|
| **BLE ELM327 OBD-II adapter** | Plugs into the Range Rover's OBD-II port. See [compatible adapters](#compatible-ble-obd-adapters). |
| **Bluetooth radio** | Either the HA host's built-in Bluetooth, or an [ESPHome Bluetooth proxy](https://esphome.io/components/bluetooth_proxy.html). |

### Compatible BLE OBD adapters

| Adapter | Notes |
|---|---|
| Vgate iCar Pro BLE | Cheap, widely available, generic ELM327 v2.1 |
| Vgate vLinker MC+ | Better STN2120 chip, supports MS-CAN |
| OBDLink CX | Premium BT 5.1, fast and reliable |
| Generic ELM327 BLE | Various — check it supports BLE (not classic BT) |

### Software

- Home Assistant **2023.6** or later
- [HACS](https://hacs.xyz/) (for HACS installation)

---

## Installation

### Option 1 — HACS (Recommended)

1. Open **HACS** in Home Assistant.
2. Go to **Integrations** → three-dot menu (⋮) → **Custom repositories**.
3. Paste: `https://github.com/ptorsten/range-rover-ha-ble`, category **Integration**. Click **Add**.
4. Find **Range Rover BLE** in the HACS list and click **Download**.
5. Restart Home Assistant.
6. Go to **Settings → Devices & Services → Add Integration**, search for **Range Rover BLE**.

### Option 2 — Manual

1. Copy the `custom_components/range_rover_ble` folder into your HA `config/custom_components/` directory.
2. Restart Home Assistant.
3. Add the integration via **Settings → Devices & Services**.

---

## JLR OBD-II Protocol Details

The Range Rover P550e uses **ISO 15765-4 CAN** (11-bit, 500 kbps) on the standard OBD-II port.

### Key ECU addresses

| ECU | Request | Response | Description |
|---|---|---|---|
| PCM | 0x7E0 | 0x7E8 | Powertrain Control Module |
| BBM | 0x7E2 | 0x7EA | Battery Backup Module |
| **BECM** | **0x7E4** | **0x7EC** | **Battery Energy Control Module** (primary target) |
| BCCM | 0x7E5 | 0x7ED | Battery Charge Control Module |

### Battery PIDs (UDS Mode 22 — BECM at 7E4)

| DID | Parameter | Formula | Unit |
|---|---|---|---|
| 0x4910 | SOC Average | (A×256+B)/100 | % |
| 0x4911 | SOC Minimum | (A×256+B)/100 | % |
| 0x490F | HV Voltage | (A×256+B)/100 | V |
| 0x490C | HV Current | (A×256+B-32768)/40 | A |
| 0x4918 | SOH Average | A/2 | % |
| 0x4903 | Cell V Max | (A×256+B)/1000 | V |
| 0x4904 | Cell V Min | (A×256+B)/1000 | V |
| 0x4905 | Battery Temp | A/2-40 | °C |

> **Note:** These DIDs are from Jaguar I-Pace research (same JLR BECM architecture). The P550e uses the newer MLA-Flex platform — DIDs may differ and must be validated on the actual vehicle.

---

## Discovery Approach

To validate and discover the correct PIDs for your specific vehicle:

1. Plug a BLE OBD adapter into the P550e with ignition on
2. The integration will attempt all configured PIDs and log responses
3. Check HA logs for `custom_components.range_rover_ble` at debug level
4. PIDs that return "NO DATA" need investigation — use `overrides.yaml` to try alternatives
5. The standard OBD PID `015B` (hybrid battery remaining life) is also attempted as a fallback

```yaml
# configuration.yaml — enable debug logging
logger:
  default: warning
  logs:
    custom_components.range_rover_ble: debug
```

### Discovery script

`scripts/discover.py` talks to the adapter directly from a laptop (needs `pip install bleak`).
On macOS the adapter shows up with a CoreBluetooth UUID instead of a MAC; pass that UUID as-is.

```bash
python3 scripts/discover.py                      # scan, pick adapter, probe the known PID list
python3 scripts/discover.py <addr>               # same, for a specific adapter
python3 scripts/discover.py --test               # offline self-test against a simulated car
```

**Finding charging status.** No known DID for charge state has been confirmed yet, so sweep the
battery (7E4), charger (7E5) and hybrid (7E6) ECUs in two states and diff them. DIDs whose bytes
change are your charging-status / charge-power candidates:

```bash
python3 scripts/discover.py <addr> --sweep --label charging  --out charging.json
python3 scripts/discover.py <addr> --sweep --label unplugged --out unplugged.json
python3 scripts/discover.py --compare charging.json unplugged.json
```

Default ranges are `4900-49FF`, `D900-D9FF`, `DD00-DDFF`; widen with `--range 0000-FFFF --ecu 7E5`
(a full sweep of one ECU takes hours over BLE).

### Validated on a P550e (2026-09)

| Request | Result |
|---|---|
| `7E4 224910/11/14` | SOC avg/min/max, `(A*256+B)/100` % — matches standard PID `015B` |
| `7E4 22490F` | HV pack voltage `/100` V (~453 V at 96 % SOC, ~110 cells in series) |
| `7E4 224903/04` | Cell voltage max/min, mV |
| `7E4 224918/19/1A` | SOH avg / min / max, `/2` % |
| `7E4 22492B/2C` | Battery plate temperatures, `A-40` °C |
| `7E4 22DD04/05` | Track ambient temperature, not SOC |
| `7E4 22DD06` | Read `0x00` awake and idle, `0x04` while AC charging. Used as an experimental charge indicator until a DID sweep confirms it |
| `7E4 22490C`, `224905`, `22491B/1C`, `22DD07/0A/0B` | Rejected: `7F 22 31` requestOutOfRange |
| Display vs raw SOC | Dashboard showed 49–50 % while raw SOC was 57.3–57.4 % (2026-10-03); the display uses a remapped usable window |
| Anything on `7E5` (BCCM) or `7E0` (PCM) | Rejected on the DIDs tried so far |

---

## Overriding OBD Commands

Create `config/custom_components/range_rover_ble/overrides.yaml` to customise or add PIDs:

```yaml
_all_:
  commands:
    # Override the SOC DID if the P550e uses a different one
    state_of_charge:
      command: "22DDXX"
      header: "7E4"

    # Disable a command that causes issues
    soc_std:
      enabled: false

    # Add a new sensor
    coolant_temp:
      command: "22491B"
      header: "7E4"
      decoder:
        type: linear_scale
        byte_offset: 3
        scale: 1
        offset: -40
      sensor:
        name: "Coolant Temperature"
        unit: "°C"
        device_class: temperature
        state_class: measurement
```

---

## Known Limitations

- **Unvalidated PIDs** — the JLR-specific PIDs need testing on an actual P550e
- **Security access** — some BECM data may require a UDS Security Access (0x27) handshake
- **Car must be on** — the BECM only responds with ignition on or in accessory mode
- **Single CAN bus** — only HS-CAN is accessible via OBD-II; some data lives on other internal buses

---

## License

This project is licensed under the [MIT License](LICENSE).

#!/usr/bin/env python3
"""Range Rover P550e PHEV — BLE OBD-II PID discovery script.

Scans for BLE OBD adapters, connects, initialises ELM327, and probes
JLR BECM/BCCM DIDs to find which ones the vehicle responds to.

Supports probing with the car off (e.g. while charging) by attempting
CAN bus wakeup and UDS TesterPresent keepalives.

Usage:
    pip install bleak
    python3 scripts/discover.py                  # scan + auto-pick adapter
    python3 scripts/discover.py AA:BB:CC:DD:EE:FF   # connect to specific address

    # Find charging-related DIDs: sweep while charging, sweep again unplugged, diff.
    python3 scripts/discover.py <addr> --sweep --label charging  --out charging.json
    python3 scripts/discover.py <addr> --sweep --label unplugged --out unplugged.json
    python3 scripts/discover.py --compare charging.json unplugged.json
    python3 scripts/discover.py --test                # offline self-test, no car needed
    python3 scripts/discover.py <addr> --raw          # ask the adapter how it sleeps (STI, STSLCS, ATPPS…)
    python3 scripts/discover.py <addr> --raw ATRV STSLU   # send your own commands
    # Narrow/widen: --ecu 7E5 --range 0000-FFFF   (full sweep of one ECU, ~hours)

Note on addresses: on Linux/Windows the address is the adapter's Bluetooth
MAC. On macOS, CoreBluetooth never exposes MACs; it gives each peripheral a
per-Mac random UUID instead, and that UUID is what this script prints and
what you pass on the command line. The Home Assistant integration does its
own scanning, so you never need the MAC for HA setup.
"""

import asyncio
import platform
import sys

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("Install bleak first:  pip install bleak")
    sys.exit(1)

# --- Known BLE OBD adapter GATT UUIDs ---
# We try these in order until one works.
UUID_CANDIDATES = [
    {
        "name": "Generic ELM327 (FFE0/FFE1)",
        "service": "0000ffe0-0000-1000-8000-00805f9b34fb",
        "notify": "0000ffe1-0000-1000-8000-00805f9b34fb",
        "write": "0000ffe1-0000-1000-8000-00805f9b34fb",
    },
    {
        "name": "OBDLink CX / FFF0 type",
        "service": "0000fff0-0000-1000-8000-00805f9b34fb",
        "notify": "0000fff1-0000-1000-8000-00805f9b34fb",
        "write": "0000fff2-0000-1000-8000-00805f9b34fb",
    },
    {
        "name": "iOS-Vlink type",
        "service": "e7810a71-73ae-499d-8c15-faa9aef0c3f2",
        "notify": "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f",
        "write": "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f",
    },
]

# --- ELM327 init ---
ELM_INIT = [
    ("ATZ", "Reset ELM327", 3.0),
    ("ATE0", "Echo off", 2.0),
    ("ATL0", "Linefeeds off", 1.0),
    ("ATS0", "Spaces off", 1.0),
    ("ATH1", "Headers on", 1.0),
    ("ATSP6", "Protocol: ISO 15765-4 CAN 11-bit 500k", 2.0),
    ("ATCAF1", "CAN auto-formatting on", 1.0),
    ("ATSTFF", "Set timeout to max (255 × 4ms = 1s)", 1.0),
    ("ATDP", "Report current protocol", 2.0),
]

# --- CAN bus wakeup / ECU wake sequence ---
# When the car is off (but potentially charging), the CAN gateway and ECUs
# may be in sleep mode. We try several approaches to wake them.
WAKEUP_SEQUENCE = [
    # 1. Send a standard OBD request (may wake the gateway)
    ("7DF", "0100", "Wake: OBD broadcast to gateway"),
    # 2. TesterPresent to BECM — UDS service 0x3E, sub-function 0x00
    #    This tells the ECU "a diagnostic tool is connected, stay awake"
    ("7E4", "3E00", "Wake: TesterPresent to BECM"),
    # 3. TesterPresent to BCCM (charge controller)
    ("7E5", "3E00", "Wake: TesterPresent to BCCM"),
    # 4. TesterPresent to PCM
    ("7E0", "3E00", "Wake: TesterPresent to PCM"),
    # 5. DiagnosticSessionControl — request extended diagnostic session
    #    Service 0x10, sub-function 0x03 = extended session
    ("7E4", "1003", "Wake: ExtendedDiagSession to BECM"),
    ("7E5", "1003", "Wake: ExtendedDiagSession to BCCM"),
]

# --- PIDs to probe ---
# (header, command, label, decoder_hint)
PIDS_TO_PROBE = [
    # Standard OBD-II (broadcast 7DF)
    ("7DF", "0100", "Supported PIDs [01-20]", None),
    ("7DF", "0120", "Supported PIDs [21-40]", None),
    ("7DF", "0140", "Supported PIDs [41-60]", None),
    ("7DF", "015B", "Hybrid battery remaining life (std)", "soc_std"),
    ("7DF", "0142", "Control module voltage (12V)", "bat_12v"),
    ("7DF", "0146", "Ambient air temperature", "ambient"),
    ("7DF", "010D", "Vehicle speed", "speed"),
    ("7DF", "010C", "Engine RPM", "rpm"),
    ("7DF", "0105", "Engine coolant temperature", "coolant"),

    # BECM — Battery Energy Control Module (7E4)
    ("7E4", "224910", "BECM: SOC average", "soc_jlr"),
    ("7E4", "224911", "BECM: SOC minimum cell", "soc_jlr"),
    ("7E4", "224914", "BECM: SOC maximum cell", "soc_jlr"),
    ("7E4", "22490F", "BECM: HV battery voltage", "hv_v"),
    ("7E4", "22490C", "BECM: HV battery current", "hv_a"),
    ("7E4", "224905", "BECM: Battery temperature", "hv_temp"),
    ("7E4", "224918", "BECM: SOH average", "soh"),
    ("7E4", "224919", "BECM: SOH min? (reads lowest)", "soh"),
    ("7E4", "22491A", "BECM: SOH max? (reads highest)", "soh"),
    ("7E4", "224903", "BECM: Cell voltage max", "cell_v"),
    ("7E4", "224904", "BECM: Cell voltage min", "cell_v"),
    ("7E4", "22491B", "BECM: Coolant outlet temp", "temp_offset"),
    ("7E4", "22491C", "BECM: Coolant inlet temp", "temp_offset"),
    ("7E4", "22492B", "BECM: Battery plate temp 1", "temp_offset"),
    ("7E4", "22492C", "BECM: Battery plate temp 2", "temp_offset"),

    # BCCM — Battery Charge Control Module (7E5)
    # This ECU manages charging and may be awake even when the car is off
    ("7E5", "224910", "BCCM: SOC (if mirrored)", "soc_jlr"),
    ("7E5", "22490F", "BCCM: HV voltage (if mirrored)", "hv_v"),
    ("7E5", "22490C", "BCCM: HV current (if mirrored)", "hv_a"),
    ("7E5", "224905", "BCCM: Battery temp (if mirrored)", "hv_temp"),
    ("7E5", "224918", "BCCM: SOH (if mirrored)", "soh"),

    # PCM — Powertrain Control Module (7E0)
    ("7E0", "224910", "PCM: SOC (if mirrored)", "soc_jlr"),
    ("7E0", "2142", "PCM: Control module voltage", None),

    # Alternative DID ranges (common in newer JLR / MLA-Flex)
    ("7E4", "22DD04", "BECM: DD04 (tracks ambient temp?)", "temp_offset"),
    ("7E4", "22DD05", "BECM: DD05 (tracks ambient temp?)", "temp_offset"),
    ("7E4", "22DD06", "BECM: HV current (alt DID DD06)", None),
    ("7E4", "22DD07", "BECM: Battery temp (alt DID DD07)", None),
    ("7E4", "22DD0A", "BECM: Charging status (alt DID DD0A)", None),
    ("7E4", "22DD0B", "BECM: EV range (alt DID DD0B)", None),

    # Same alt DIDs on BCCM
    ("7E5", "22DD04", "BCCM: DD04 (tracks ambient temp?)", "temp_offset"),
    ("7E5", "22DD0A", "BCCM: Charging status (alt DID DD0A)", None),
    ("7E5", "22DD0B", "BCCM: EV range (alt DID DD0B)", None),
]


def _parse_frames(raw_hex: str) -> list[tuple[str, bytes]]:
    """Split an ELM327 response (ATH1, ATS0) into (can_id, payload) pairs.

    With headers on and spaces off, each frame looks like "7EC037F2231":
    a 3-hex-digit 11-bit CAN ID, a PCI byte (0x0N = single frame, N data
    bytes), then the data. The odd total length is what broke a naive
    bytes.fromhex() on the whole string.
    """
    frames = []
    for frame in raw_hex.split():
        frame = frame.strip()
        if len(frame) % 2 == 1 and len(frame) >= 5:
            can_id, body = frame[:3], frame[3:]
        elif len(frame) >= 10 and frame[:2] == "18":  # 29-bit header, 8 hex digits
            can_id, body = frame[:8], frame[8:]
        else:
            can_id, body = "", frame
        try:
            data = bytes.fromhex(body)
        except ValueError:
            continue
        if not data:
            continue
        pci = data[0]
        if pci >> 4 == 0:            # single frame: low nibble = length
            data = data[1 : 1 + (pci & 0x0F)]
        elif pci >> 4 == 1 and len(data) > 1:  # first frame of multi-frame
            data = data[2:]
        elif pci >> 4 == 2:          # consecutive frame: 1 PCI byte
            data = data[1:]
        frames.append((can_id, data))
    return frames


def _reassemble(raw_hex: str) -> dict[str, bytes]:
    """Concatenate all frames per CAN ID, in order received."""
    out: dict[str, bytes] = {}
    for can_id, data in _parse_frames(raw_hex):
        out[can_id] = out.get(can_id, b"") + data
    return out


def _payload(raw_hex: str) -> bytes:
    """Return the payload of the first frame that is not a negative response.

    Falls back to the first frame if every ECU answered with 7F.
    """
    frames = _parse_frames(raw_hex)
    for _, data in frames:
        if data and data[0] != 0x7F:
            return data
    return frames[0][1] if frames else b""


def try_decode(raw_hex: str, decoder: str | None) -> str:
    """Attempt to decode a raw hex response into a human-readable value."""
    if not decoder or not raw_hex:
        return ""
    data = _payload(raw_hex)
    if not data:
        return ""

    try:
        if decoder == "soc_jlr":
            i = _find_byte(data, 0x62)
            if i is not None and i + 4 < len(data):
                val = (data[i + 3] * 256 + data[i + 4]) / 100.0
                return f"= {val:.1f} %"
        elif decoder == "soc_std":
            i = _find_byte(data, 0x41)
            if i is not None and i + 2 < len(data):
                val = data[i + 2] * 100.0 / 255.0
                return f"= {val:.1f} %"
        elif decoder == "soc_alt":
            i = _find_byte(data, 0x62)
            if i is not None and i + 3 < len(data):
                val = data[i + 3] * 100.0 / 255.0
                return f"= {val:.1f} % (single-byte)"
        elif decoder == "bat_12v":
            i = _find_byte(data, 0x41)
            if i is not None and i + 3 < len(data):
                val = (data[i + 2] * 256 + data[i + 3]) / 1000.0
                return f"= {val:.2f} V"
        elif decoder == "hv_v":
            i = _find_byte(data, 0x62)
            if i is not None and i + 4 < len(data):
                val = (data[i + 3] * 256 + data[i + 4]) / 100.0
                return f"= {val:.1f} V"
        elif decoder == "hv_a":
            i = _find_byte(data, 0x62)
            if i is not None and i + 4 < len(data):
                val = (data[i + 3] * 256 + data[i + 4] - 32768) / 40.0
                return f"= {val:.2f} A"
        elif decoder == "hv_temp":
            i = _find_byte(data, 0x62)
            if i is not None and i + 3 < len(data):
                val = data[i + 3] / 2.0 - 40.0
                return f"= {val:.0f} °C"
        elif decoder == "soh":
            i = _find_byte(data, 0x62)
            if i is not None and i + 3 < len(data):
                val = data[i + 3] / 2.0
                return f"= {val:.1f} %"
        elif decoder == "cell_v":
            i = _find_byte(data, 0x62)
            if i is not None and i + 4 < len(data):
                val = (data[i + 3] * 256 + data[i + 4]) / 1000.0
                return f"= {val:.3f} V"
        elif decoder == "temp_offset":
            i = _find_byte(data, 0x62)
            if i is not None and i + 3 < len(data):
                val = data[i + 3] - 40
                return f"= {val} °C"
        elif decoder == "ambient":
            i = _find_byte(data, 0x41)
            if i is not None and i + 2 < len(data):
                val = data[i + 2] - 40
                return f"= {val} °C"
        elif decoder == "speed":
            i = _find_byte(data, 0x41)
            if i is not None and i + 2 < len(data):
                return f"= {data[i+2]} km/h"
        elif decoder == "rpm":
            i = _find_byte(data, 0x41)
            if i is not None and i + 3 < len(data):
                val = (data[i + 2] * 256 + data[i + 3]) / 4.0
                return f"= {val:.0f} RPM"
        elif decoder == "coolant":
            i = _find_byte(data, 0x41)
            if i is not None and i + 2 < len(data):
                return f"= {data[i+2] - 40} °C"
    except Exception:
        pass
    return ""


def _find_byte(data: bytes, target: int) -> int | None:
    for i in range(len(data)):
        if data[i] == target:
            return i
    return None


def _classify_nrc(raw: str) -> str | None:
    """If the response is a UDS Negative Response (7F), decode the NRC."""
    frames = _parse_frames(raw)
    if not frames:
        return None
    # Only a negative response if *every* responding ECU said 7F; on a
    # broadcast (7DF) one ECU may reject while another answers.
    if any(data and data[0] != 0x7F for _, data in frames):
        return None
    for _, data in frames:
        if len(data) >= 3 and data[0] == 0x7F:
            service = data[1]
            nrc = data[2]
            nrc_names = {
                0x10: "generalReject",
                0x11: "serviceNotSupported",
                0x12: "subFunctionNotSupported",
                0x13: "incorrectMessageLength",
                0x14: "responseTooLong",
                0x22: "conditionsNotCorrect",
                0x24: "requestSequenceError",
                0x25: "noResponseFromSubnet",
                0x31: "requestOutOfRange",
                0x33: "securityAccessDenied",
                0x35: "invalidKey",
                0x36: "exceededNumberOfAttempts",
                0x37: "requiredTimeDelayNotExpired",
                0x70: "uploadDownloadNotAccepted",
                0x71: "transferDataSuspended",
                0x72: "generalProgrammingFailure",
                0x73: "wrongBlockSequenceCounter",
                0x78: "requestCorrectlyReceivedResponsePending",
                0x7E: "subFunctionNotSupportedInActiveSession",
                0x7F: "serviceNotSupportedInActiveSession",
            }
            name = nrc_names.get(nrc, f"0x{nrc:02X}")
            return f"NRC: {name} (service 0x{service:02X})"
    return None


PACE = 1.0  # multiplier on inter-command sleeps; 0 in --test mode


def open_client(address: str):
    """Open a BLE client. Replaced by the simulator in --test mode."""
    return BleakClient(address, timeout=20.0)


class ELM327BLE:
    """Minimal ELM327-over-BLE driver for discovery."""

    def __init__(self):
        self._buf = bytearray()
        self._evt = asyncio.Event()

    def _on_notify(self, _sender, data: bytearray):
        self._buf.extend(data)
        if b">" in data:
            self._evt.set()

    async def send(self, client: BleakClient, write_uuid: str, cmd: str, timeout: float = 5.0) -> str:
        self._buf.clear()
        self._evt.clear()
        await client.write_gatt_char(write_uuid, f"{cmd}\r".encode())
        try:
            await asyncio.wait_for(self._evt.wait(), timeout=timeout)
        except TimeoutError:
            return "<TIMEOUT>"
        resp = self._buf.decode("ascii", errors="replace").strip().replace(">", "").strip()
        return resp


async def scan_adapters() -> list:
    """Scan for BLE devices that look like OBD adapters."""
    print("\nScanning for BLE devices (10 seconds)...\n")
    # bleak >= 0.19 deprecated (and 2.x removed) BLEDevice.rssi; RSSI now
    # lives in the AdvertisementData returned alongside each device.
    found = await BleakScanner.discover(timeout=10.0, return_adv=True)

    obd_keywords = {"obd", "elm", "vgate", "vlink", "icar", "obdlink", "lelink", "car"}
    candidates = []
    candidate_info = []
    all_devices = []

    for d, adv in found.values():
        name = (d.name or adv.local_name or "").strip()
        # CoreBluetooth reports 127 when RSSI is unavailable.
        rssi = adv.rssi if adv.rssi is not None and adv.rssi != 127 else -999
        all_devices.append((d.address, name, rssi))
        if any(kw in name.lower() for kw in obd_keywords):
            candidates.append(d)
            candidate_info.append((name, rssi))

    if candidates:
        print(f"Found {len(candidates)} likely OBD adapter(s):\n")
        for i, (d, (name, rssi)) in enumerate(zip(candidates, candidate_info)):
            print(f"  [{i}] {name}  ({d.address})  RSSI: {rssi}")
    else:
        print("No obvious OBD adapters found. All BLE devices:\n")
        for addr, name, rssi in sorted(all_devices, key=lambda x: x[2], reverse=True):
            label = name if name else "(no name)"
            rssi_str = "n/a" if rssi == -999 else str(rssi)
            print(f"  {label:30s}  {addr}  RSSI: {rssi_str}")
        print("\nTip: make sure the OBD adapter is plugged in.")
        if platform.system() == "Darwin":
            print(
                "Note: macOS hides Bluetooth MAC addresses; the UUIDs above are\n"
                "CoreBluetooth identifiers and can be passed to this script as-is."
            )

    return candidates


async def _setup_adapter(client, elm: "ELM327BLE", init=None) -> tuple[str, str] | None:
    """Pick GATT UUIDs, subscribe to notifications, run ELM327 init.

    ``init`` defaults to ELM_INIT (which starts with ATZ). Pass a shorter list
    when you must not reset the adapter, e.g. while changing its settings.
    Returns (write_uuid, notify_uuid) or None if no known UUID set matched.
    """
    if init is None:
        init = ELM_INIT
    services = client.services
    print("GATT services:")
    notify_uuid = None
    write_uuid = None

    for service in services:
        print(f"  Service: {service.uuid}")
        for char in service.characteristics:
            props = ", ".join(char.properties)
            print(f"    Char: {char.uuid}  [{props}]")

    for candidate in UUID_CANDIDATES:
        for char in [c for s in services for c in s.characteristics]:
            if char.uuid.lower() == candidate["notify"].lower():
                if "notify" in char.properties or "indicate" in char.properties:
                    notify_uuid = candidate["notify"]
                    write_uuid = candidate["write"]
                    print(f"\n  Using UUID set: {candidate['name']}")
                    break
        if notify_uuid:
            break

    if not notify_uuid:
        print("\n  Could not find a matching notify/write characteristic.")
        print("  You may need to identify the correct UUIDs from the list above")
        print("  and pass them manually.")
        return None

    await client.start_notify(notify_uuid, elm._on_notify)

    print("\n--- ELM327 Initialisation ---\n")
    for cmd, desc, timeout in init:
        resp = await elm.send(client, write_uuid, cmd, timeout)
        ok = "ERROR" not in resp.upper() and "TIMEOUT" not in resp
        status = "  OK" if ok else "WARN"
        print(f"  [{status}] {cmd:10s} ({desc})")
        if resp and resp != cmd:
            for line in resp.split("\r"):
                line = line.strip()
                if line:
                    print(f"          -> {line}")
        await asyncio.sleep(0.3 * PACE)
    return write_uuid, notify_uuid


async def discover(address: str):
    """Connect to the adapter and probe all PIDs."""
    print(f"\nConnecting to {address}...\n")

    elm = ELM327BLE()

    async with open_client(address) as client:
        print(f"Connected: {client.is_connected}\n")

        uuids = await _setup_adapter(client, elm)
        if not uuids:
            return
        write_uuid, notify_uuid = uuids

        # --- CAN bus / ECU wakeup ---
        print("\n--- CAN Bus Wakeup Sequence ---")
        print("  (Attempting to wake ECUs — needed when car is off/charging)\n")

        wakeup_responded = False
        last_header = None
        for header, command, label in WAKEUP_SEQUENCE:
            if header != last_header:
                await elm.send(client, write_uuid, f"ATSH{header}", 2.0)
                last_header = header
                await asyncio.sleep(0.1 * PACE)

            resp = await elm.send(client, write_uuid, command, 3.0)
            resp_upper = resp.upper().replace("\r", " ").strip()
            await asyncio.sleep(0.3 * PACE)

            if not resp or "TIMEOUT" in resp or "NO DATA" in resp_upper:
                print(f"  [    ] {label:45s} no response")
            elif "ERROR" in resp_upper:
                print(f"  [FAIL] {label:45s} {resp_upper}")
            else:
                wakeup_responded = True
                nrc = _classify_nrc(resp_upper)
                if nrc:
                    print(f"  [NRC ] {label:45s} {nrc}")
                else:
                    print(f"  [ OK ] {label:45s} responded")
                    print(f"          raw: {resp_upper}")

        if not wakeup_responded:
            print("\n  ** No ECU responded to wakeup. Possible causes:")
            print("     - Car is fully asleep (not charging)")
            print("     - CAN gateway blocks OBD access when ignition is off")
            print("     - Wrong protocol (try ATSP7 for 29-bit CAN)")
            print("     - OBD adapter not connected or powered")
            print("\n  Continuing with PID scan anyway (some may still work)...\n")
        else:
            # Send a second round of TesterPresent after a short pause
            # to make sure ECUs stay awake during the full scan
            await asyncio.sleep(1.0 * PACE)
            for header in ("7E4", "7E5"):
                await elm.send(client, write_uuid, f"ATSH{header}", 1.0)
                await elm.send(client, write_uuid, "3E00", 2.0)
                await asyncio.sleep(0.2)
            last_header = "7E5"

        # --- Probe PIDs ---
        print("\n--- PID Discovery ---\n")
        results = {"responded": [], "no_data": [], "error": [], "nrc": []}
        tester_present_counter = 0

        for header, command, label, decoder in PIDS_TO_PROBE:
            # Set header if changed
            if header != last_header:
                await elm.send(client, write_uuid, f"ATSH{header}", 2.0)
                last_header = header
                print(f"\n  [ECU header: {header}]")
                await asyncio.sleep(0.2)

            resp = await elm.send(client, write_uuid, command, 5.0)
            await asyncio.sleep(0.3 * PACE)

            # Periodically send TesterPresent to keep ECUs awake
            tester_present_counter += 1
            if tester_present_counter >= 8:
                tester_present_counter = 0
                saved_header = last_header
                for tp_header in ("7E4", "7E5"):
                    await elm.send(client, write_uuid, f"ATSH{tp_header}", 1.0)
                    await elm.send(client, write_uuid, "3E00", 2.0)
                    await asyncio.sleep(0.1 * PACE)
                if saved_header:
                    await elm.send(client, write_uuid, f"ATSH{saved_header}", 1.0)
                    last_header = saved_header

            # Classify response
            resp_upper = resp.upper().replace("\r", " ").strip()
            if not resp or "TIMEOUT" in resp:
                status = "TIMEOUT"
                results["no_data"].append(label)
            elif "NO DATA" in resp_upper:
                status = "NO DATA"
                results["no_data"].append(label)
            elif "ERROR" in resp_upper or "?" in resp_upper:
                status = "ERROR  "
                results["error"].append(label)
            else:
                nrc = _classify_nrc(resp_upper)
                if nrc:
                    status = f"NRC     {nrc}"
                    results["nrc"].append((label, nrc))
                else:
                    decoded = try_decode(resp_upper, decoder)
                    status = f"DATA    {decoded}"
                    results["responded"].append((label, resp_upper, decoded))

            print(f"    {command:10s} {label:45s} {status}")
            # Show raw hex for responses that had data
            if "DATA" in status:
                print(f"               raw: {resp_upper}")

        await client.stop_notify(notify_uuid)

    # --- Summary ---
    print("\n" + "=" * 70)
    print("DISCOVERY SUMMARY")
    print("=" * 70)

    if results["responded"]:
        print(f"\n  {len(results['responded'])} PID(s) returned data:\n")
        for label, raw, decoded in results["responded"]:
            print(f"   + {label:50s} {decoded}")
    else:
        print("\n  No PIDs returned data.")
        print("  Possible causes:")
        print("    - Car is fully asleep (try with ignition on first)")
        print("    - CAN gateway blocks OBD when off (common in newer JLR)")
        print("    - Adapter needs different GATT UUIDs")
        print("    - PIDs are wrong for MLA-Flex platform")

    if results["nrc"]:
        print(f"\n  {len(results['nrc'])} PID(s) returned Negative Response Codes:")
        print("  (ECU is awake but rejected the request)\n")
        for label, nrc in results["nrc"]:
            print(f"   ! {label:50s} {nrc}")
        # Helpful hints based on NRC types
        nrc_texts = [n for _, n in results["nrc"]]
        if any("securityAccessDenied" in n for n in nrc_texts):
            print("\n  ** securityAccessDenied: some PIDs need UDS Security Access (0x27)")
            print("     This means the ECU is awake but requires authentication.")
            print("     Security key algorithms are vehicle-specific.")
        if any("conditionsNotCorrect" in n for n in nrc_texts):
            print("\n  ** conditionsNotCorrect: ECU can't provide data in current state")
            print("     May work when car is in a different mode (ignition on, charging, etc.)")
        if any("serviceNotSupportedInActiveSession" in n for n in nrc_texts):
            print("\n  ** Try requesting an extended diagnostic session first:")
            print("     ATSH7E4 then 1003 (ExtendedDiagSession)")

    if results["no_data"]:
        print(f"\n  {len(results['no_data'])} PID(s) returned NO DATA (ECU didn't respond)")

    if results["error"]:
        print(f"\n  {len(results['error'])} PID(s) returned ERROR:")
        for label in results["error"]:
            print(f"   x {label}")

    print()


# ---------------------------------------------------------------------------
# DID sweep: brute-force ReadDataByIdentifier (service 0x22) over DID ranges
# and record everything that answers. Run once while charging and once
# unplugged, then --compare the two JSON files: the DIDs whose bytes change
# are your charging-status / charge-power candidates.
# ---------------------------------------------------------------------------

DEFAULT_SWEEP_ECUS = ["7E4", "7E5", "7E6"]      # BECM, BCCM, third responder (7EE)
DEFAULT_SWEEP_RANGES = ["4900-49FF", "D900-D9FF", "DD00-DDFF"]


def _parse_ranges(spec: list[str]) -> list[int]:
    dids: list[int] = []
    for item in spec:
        for part in item.split(","):
            part = part.strip()
            if not part:
                continue
            if "-" in part:
                lo, hi = part.split("-", 1)
                dids.extend(range(int(lo, 16), int(hi, 16) + 1))
            else:
                dids.append(int(part, 16))
    return dids


def _hexdump_guess(payload: bytes) -> str:
    """Show a few plausible interpretations of a DID's data bytes."""
    if not payload:
        return ""
    parts = [f"u8={payload[0]}", f"u8-40={payload[0] - 40}"]
    if len(payload) >= 2:
        u16 = int.from_bytes(payload[:2], "big")
        s16 = int.from_bytes(payload[:2], "big", signed=True)
        parts += [f"u16={u16}", f"u16/100={u16 / 100:.2f}", f"s16={s16}"]
    if len(payload) >= 4:
        parts.append(f"u32={int.from_bytes(payload[:4], 'big')}")
    return "  ".join(parts)


async def sweep(address: str, ecus: list[str], ranges: list[str], out_path: str, label: str):
    """Sweep DIDs on each ECU and save all positive responses to JSON."""
    import json
    from datetime import datetime

    dids = _parse_ranges(ranges)
    print(f"\nConnecting to {address}...\n")
    elm = ELM327BLE()
    results: dict = {
        "label": label,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "ecus": {},
    }

    async with open_client(address) as client:
        print(f"Connected: {client.is_connected}\n")
        uuids = await _setup_adapter(client, elm)
        if not uuids:
            return
        write_uuid, notify_uuid = uuids

        for ecu in ecus:
            print(f"\n--- Sweeping {len(dids)} DIDs on {ecu} ({', '.join(ranges)}) ---\n")
            await elm.send(client, write_uuid, f"ATSH{ecu}", 1.0)
            # Wake + extended session: some DIDs are only readable in 0x03.
            tp = await elm.send(client, write_uuid, "3E00", 2.0)
            if "TIMEOUT" in tp or "NO DATA" in tp.upper():
                print(f"  {ecu}: no answer to TesterPresent, skipping")
                continue
            await elm.send(client, write_uuid, "1003", 2.0)

            hits: dict[str, dict] = {}
            nrc_other: dict[str, str] = {}
            t0 = asyncio.get_running_loop().time()
            for n, did in enumerate(dids):
                if n and n % 25 == 0:
                    await elm.send(client, write_uuid, "3E00", 1.0)   # keep session alive
                    done = n / len(dids)
                    elapsed = asyncio.get_running_loop().time() - t0
                    eta = elapsed / done - elapsed if done else 0
                    print(f"  ... {n}/{len(dids)}  ({len(hits)} hits, ~{eta:.0f}s left)", end="\r", flush=True)

                cmd = f"22{did:04X}"
                resp = await elm.send(client, write_uuid, cmd, 1.5)
                resp_upper = resp.upper().replace("\r", " ").strip()
                if not resp or "TIMEOUT" in resp or "NO DATA" in resp_upper or "ERROR" in resp_upper:
                    continue

                # 0x78 = response pending; the ELM usually delivers the real
                # answer in the same read, but retry once if not.
                if "7F2278" in resp_upper.replace(" ", "") and f"62{did:04X}" not in resp_upper.replace(" ", ""):
                    await asyncio.sleep(0.5 * PACE)
                    resp = await elm.send(client, write_uuid, cmd, 2.0)
                    resp_upper = resp.upper().replace("\r", " ").strip()

                frames = _reassemble(resp_upper)
                positive = None
                for can_id, data in frames.items():
                    if len(data) >= 3 and data[0] == 0x62 and int.from_bytes(data[1:3], "big") == did:
                        positive = (can_id, data[3:])
                        break
                if positive:
                    can_id, payload = positive
                    key = f"{did:04X}"
                    hits[key] = {"from": can_id, "hex": payload.hex().upper(), "raw": resp_upper}
                    print(f"  {' ' * 60}", end="\r")
                    print(f"  22{key}  <- {can_id}  [{len(payload):2d}B]  {payload.hex().upper():24s}  {_hexdump_guess(payload)}")
                else:
                    nrc = _classify_nrc(resp_upper)
                    if nrc and "requestOutOfRange" not in nrc:
                        nrc_other[f"{did:04X}"] = nrc

            print(f"  {' ' * 60}", end="\r")
            print(f"  {ecu}: {len(hits)} DID(s) answered, {len(nrc_other)} other NRC(s)")
            for k, v in sorted(nrc_other.items()):
                print(f"    22{k}: {v}")
            results["ecus"][ecu] = {"hits": hits, "nrc": nrc_other}

        await client.stop_notify(notify_uuid)

    with open(out_path, "w") as f:
        json.dump(results, f, indent=1)
    print(f"\nSaved to {out_path}")
    print("Run again in the other state (plugged/unplugged, charging/idle) with a")
    print("different --out, then:  python3 scripts/discover.py --compare a.json b.json")


def compare(path_a: str, path_b: str):
    """Diff two sweep files and highlight DIDs whose data changed."""
    import json

    with open(path_a) as f:
        a = json.load(f)
    with open(path_b) as f:
        b = json.load(f)
    print(f"\nA = {path_a}  ({a.get('label') or a.get('timestamp')})")
    print(f"B = {path_b}  ({b.get('label') or b.get('timestamp')})\n")

    for ecu in sorted(set(a["ecus"]) | set(b["ecus"])):
        ha = a["ecus"].get(ecu, {}).get("hits", {})
        hb = b["ecus"].get(ecu, {}).get("hits", {})
        changed = [(k, ha[k]["hex"], hb[k]["hex"]) for k in sorted(set(ha) & set(hb)) if ha[k]["hex"] != hb[k]["hex"]]
        same = len(set(ha) & set(hb)) - len(changed)
        only_a = sorted(set(ha) - set(hb))
        only_b = sorted(set(hb) - set(ha))
        print(f"[{ecu}]  {same} unchanged, {len(changed)} changed, {len(only_a)} only in A, {len(only_b)} only in B")
        for k, x, y in changed:
            px, py = bytes.fromhex(x), bytes.fromhex(y)
            print(f"  22{k}  A={x:24s} B={y:24s}")
            print(f"          A: {_hexdump_guess(px)}")
            print(f"          B: {_hexdump_guess(py)}")
        for k in only_a:
            print(f"  22{k}  only in A: {ha[k]['hex']}")
        for k in only_b:
            print(f"  22{k}  only in B: {hb[k]['hex']}")
        print()

# ---------------------------------------------------------------------------
# --test mode: an in-process ELM327/vehicle simulator replaying the responses
# recorded from the real P550e on 2026-09-26, plus unit checks on the parsers.
# Lets you exercise discover / sweep / compare with no adapter and no car.
# ---------------------------------------------------------------------------

# Real responses from the car (header, command) -> raw ELM output (ATH1, ATS0).
RECORDED_RESPONSES: dict[tuple[str, str], str] = {
    ("7DF", "0100"): "7ED06410080000001 7EC06410098188001 7EE06410098188001",
    ("7DF", "0120"): "7EC06412000018001 7ED06412000000001 7EE0641208001A001",
    ("7DF", "0140"): "7EC064140C4000021 7ED064140C0000000 7EE064140C4800000",
    ("7DF", "015B"): "7EC03415BF5",
    ("7DF", "0142"): "7EC0441423333 7ED0441423414 7EE044142344A",
    ("7DF", "0146"): "7EC03414637 7EE03414637",
    ("7DF", "010D"): "7EC03410D00 7EE03410D00",
    ("7DF", "010C"): "7EC04410C0000 7EE04410C0000",
    ("7DF", "0105"): "7EC03410537 7EE03410537",
    ("7E4", "3E00"): "7EC027E00",
    ("7E5", "3E00"): "7ED027E00",
    ("7E0", "3E00"): "7E8027E00",
    ("7E6", "3E00"): "7EE027E00",
    ("7E4", "1003"): "7EC065003001901F4",
    ("7E5", "1003"): "7ED065003001901F4",
    ("7E6", "1003"): "7EE065003001901F4",
    ("7E4", "224910"): "7EC05624910258C",
    ("7E4", "224911"): "7EC056249112582",
    ("7E4", "224914"): "7EC05624914258D",
    ("7E4", "22490F"): "7EC0562490FB111",
    ("7E4", "224918"): "7EC04624918BF",
    ("7E4", "224919"): "7EC04624919B6",
    ("7E4", "22491A"): "7EC0462491AC2",
    ("7E4", "224903"): "7EC05624903101E",
    ("7E4", "224904"): "7EC05624904100D",
    ("7E4", "22492B"): "7EC0462492B3D",
    ("7E4", "22492C"): "7EC0462492C3F",
    ("7E4", "22DD04"): "7EC0462DD0436",
    ("7E4", "22DD05"): "7EC0462DD0537",
    ("7E4", "22DD06"): "7EC0462DD0600",
    ("7E5", "22DD04"): "7ED0462DD0436",
    ("7E0", "2142"): "7E8037F2134",
}

# SIMULATED (not recorded) values used only to demonstrate --sweep/--compare
# finding a DID that changes between two states. 22DD06 really exists and
# read 0x00 with the car awake; what it does when charging is unknown.
SIMULATED_CHARGING_OVERRIDES: dict[tuple[str, str], str] = {
    ("7E4", "22DD06"): "7EC0462DD061C",
    ("7E4", "224910"): "7EC056249102596",
}

RESPONSE_ID = {"7E0": "7E8", "7E4": "7EC", "7E5": "7ED", "7E6": "7EE", "7DF": "7EC"}


class _FakeChar:
    def __init__(self, uuid, props):
        self.uuid, self.properties = uuid, props


class _FakeService:
    def __init__(self, uuid, chars):
        self.uuid, self.characteristics = uuid, chars


class FakeElmClient:
    """Stands in for BleakClient: ELM327 v2.2 in front of the recorded car."""

    def __init__(self, address: str, overrides: dict | None = None):
        self.address = address
        self.is_connected = False
        self._cb = None
        self._header = "7DF"
        self._table = dict(RECORDED_RESPONSES)
        if overrides:
            self._table.update(overrides)
        # Mirror the vLinker's GATT layout (matches the "iOS-Vlink type" set).
        vl = UUID_CANDIDATES[[c["name"] for c in UUID_CANDIDATES].index("iOS-Vlink type")]
        self.services = [
            _FakeService("0000180a-0000-1000-8000-00805f9b34fb",
                         [_FakeChar("00002a29-0000-1000-8000-00805f9b34fb", ["read"])]),
            _FakeService(vl["service"],
                         [_FakeChar(vl["notify"], ["indicate", "read", "notify", "write", "write-without-response"])]),
        ]
        self.sent: list[str] = []

    async def __aenter__(self):
        self.is_connected = True
        return self

    async def __aexit__(self, *exc):
        self.is_connected = False

    async def start_notify(self, _uuid, cb):
        self._cb = cb

    async def stop_notify(self, _uuid):
        self._cb = None

    def _respond(self, cmd: str) -> str:
        c = cmd.upper()
        if c == "ATZ":
            return "ELM327 v2.2"
        if c == "ATDP":
            return "ISO 15765-4 (CAN 11/500)"
        if c.startswith("ATSH"):
            self._header = c[4:]
            return "OK"
        if c.startswith("AT"):
            return "OK"
        if (self._header, c) in self._table:
            return self._table[(self._header, c)]
        rid = RESPONSE_ID.get(self._header)
        if rid is None:
            return "NO DATA"
        if c.startswith("22") and len(c) == 6:
            return f"{rid}037F2231"          # requestOutOfRange
        if c.startswith("01") and self._header == "7DF":
            return "NO DATA"
        return f"{rid}037F{c[:2]}11"         # serviceNotSupported

    async def write_gatt_char(self, _uuid, data: bytes):
        cmd = data.decode("ascii", errors="replace").strip()
        self.sent.append(cmd)
        resp = self._respond(cmd)
        if self._cb:
            self._cb(None, bytearray(f"{resp}\r\r>".encode()))


def _unit_checks() -> list[str]:
    """Parser/decoder assertions against the recorded frames. Returns failures."""
    fails: list[str] = []

    def eq(name, got, exp):
        if got != exp:
            fails.append(f"{name}: got {got!r}, expected {exp!r}")

    eq("nrc single", _classify_nrc("7EC037F2231"), "NRC: requestOutOfRange (service 0x22)")
    eq("nrc pcm 0x34", _classify_nrc("7E8037F2134"), "NRC: 0x34 (service 0x21)")
    eq("positive not nrc", _classify_nrc("7EC05624910258C"), None)
    eq("broadcast mixed", _classify_nrc("7ED037F2231 7EC05624910258C"), None)
    eq("soc", try_decode("7EC05624910258C", "soc_jlr"), "= 96.1 %")
    eq("soc std", try_decode("7EC03415BF5", "soc_std"), "= 96.1 %")
    eq("hv v", try_decode("7EC0562490FB111", "hv_v"), "= 453.3 V")
    eq("cell v", try_decode("7EC05624903101E", "cell_v"), "= 4.126 V")
    eq("soh", try_decode("7EC04624918BF", "soh"), "= 95.5 %")
    eq("plate temp", try_decode("7EC0462492B3D", "temp_offset"), "= 21 °C")
    eq("12v multi-ecu", try_decode("7EC0441423333 7ED0441423414 7EE044142344A", "bat_12v"), "= 13.11 V")
    eq("ambient", try_decode("7EC03414637 7EE03414637", "ambient"), "= 15 °C")
    eq("nrc decodes to nothing", try_decode("7EC037F2231", "hv_a"), "")
    eq("frames", _parse_frames("7EC05624910258C"), [("7EC", bytes.fromhex("624910258C"))])
    eq("multi-frame reassembly",
       _reassemble("7EC1014624910AABBCC 7EC21DDEEFF00112233 7EC22445566"),
       {"7EC": bytes.fromhex("624910AABBCCDDEEFF00112233445566")})
    eq("ranges", _parse_ranges(["4900-4902,DD06"]), [0x4900, 0x4901, 0x4902, 0xDD06])
    eq("guess", _hexdump_guess(bytes.fromhex("258C")).split("  ")[3], "u16/100=96.12")
    return fails


async def selftest() -> int:
    """Run unit checks, then discover + sweep + compare against the simulator."""
    global open_client, PACE
    import contextlib
    import io
    import json
    import os
    import tempfile

    print("=" * 70)
    print("SELF-TEST  (simulated ELM327 + recorded P550e responses, no BLE)")
    print("=" * 70)

    fails = _unit_checks()
    print(f"\n[1/4] parser unit checks: {'OK' if not fails else 'FAIL'}")
    for f in fails:
        print(f"      - {f}")

    PACE = 0.0
    state = {"overrides": None}
    open_client = lambda address: FakeElmClient(address, state["overrides"])  # noqa: E731

    # 2. discover()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        await discover("SIMULATED")
    out = buf.getvalue()
    checks = {
        "picks vLinker UUID set": "Using UUID set: iOS-Vlink type" in out,
        "SOC decoded": "BECM: SOC average" in out and "= 96.1 %" in out,
        "HV voltage decoded": "= 453.3 V" in out,
        "HV current flagged as NRC": any("HV battery current" in l and "NRC" in l for l in out.splitlines()),
        "BCCM DIDs flagged as NRC": any("BCCM: SOC (if mirrored)" in l and "NRC" in l for l in out.splitlines()),
        "PCM 2142 flagged as NRC": any("PCM: Control module voltage" in l and "NRC" in l for l in out.splitlines()),
    }
    ok2 = all(checks.values())
    print(f"[2/4] discover() against simulator: {'OK' if ok2 else 'FAIL'}")
    for k, v in checks.items():
        if not v:
            print(f"      - {k}")
            fails.append(f"discover: {k}")
    if not ok2:
        print(out)

    # 3. sweep() twice (idle, then simulated charging) on a small range
    tmp = tempfile.mkdtemp()
    a, b = os.path.join(tmp, "idle.json"), os.path.join(tmp, "charging.json")
    with contextlib.redirect_stdout(io.StringIO()):
        await sweep("SIMULATED", ["7E4", "7E5"], ["4900-4920", "DD00-DD0F"], a, "idle")
        state["overrides"] = SIMULATED_CHARGING_OVERRIDES
        await sweep("SIMULATED", ["7E4", "7E5"], ["4900-4920", "DD00-DD0F"], b, "charging")
    with open(a) as f:
        ja = json.load(f)
    hits = ja["ecus"]["7E4"]["hits"]
    checks = {
        "sweep found 224910": "4910" in hits and hits["4910"]["hex"] == "258C",
        "sweep found 22DD06": "DD06" in hits and hits["DD06"]["hex"] == "00",
        "sweep skipped NRC DIDs": "490C" not in hits,
        "sweep hit count": len(hits) == 12,  # 492B/492C are outside 4900-4920
        "BCCM sweep": ja["ecus"]["7E5"]["hits"].get("DD04", {}).get("hex") == "36",
    }
    ok3 = all(checks.values())
    print(f"[3/4] sweep() idle + charging: {'OK' if ok3 else 'FAIL'}")
    for k, v in checks.items():
        if not v:
            print(f"      - {k}  (hits={sorted(hits)})")
            fails.append(f"sweep: {k}")

    # 4. compare()
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        compare(a, b)
    out = buf.getvalue()
    checks = {
        "reports 2 changed on 7E4": "[7E4]  10 unchanged, 2 changed" in out,
        "flags 22DD06": "22DD06  A=00" in out,
        "flags 224910": "224910  A=258C" in out,
        "7E5 unchanged": "[7E5]  1 unchanged, 0 changed" in out,
    }
    ok4 = all(checks.values())
    print(f"[4/4] compare(): {'OK' if ok4 else 'FAIL'}")
    for k, v in checks.items():
        if not v:
            print(f"      - {k}")
            fails.append(f"compare: {k}")
    if not ok4:
        print(out)

    print()
    if fails:
        print(f"SELF-TEST FAILED: {len(fails)} problem(s)")
        return 1
    print("SELF-TEST PASSED")
    print(f"\nSimulated sweep files kept in {tmp} — try:")
    print(f"  python3 scripts/discover.py --compare {a} {b}")
    return 0

# Commands worth asking an adapter to learn how it sleeps. "ST" commands are
# answered only by STN-based adapters (OBDLink, some Vgate models); a plain
# ELM327 clone replies "?" to them.
SLEEP_PROBE_COMMANDS = [
    ("ATI", "ELM327 identity"),
    ("AT@1", "device description"),
    ("ATRV", "battery voltage seen by adapter"),
    ("STI", "STN firmware (STN chips only)"),
    ("STDI", "STN device id"),
    ("STSLCS", "STN sleep/wake config summary"),
    ("ATPPS", "ELM327 programmable parameters (PP 0E/0F = low power)"),
]


async def raw(address: str, commands: list[str]):
    """Send arbitrary AT/ST/OBD commands and print the replies."""
    print(f"\nConnecting to {address}...\n")
    elm = ELM327BLE()
    async with open_client(address) as client:
        print(f"Connected: {client.is_connected}\n")
        # No ATZ here: a reset would apply/undo programmable-parameter changes
        # mid-session. Just turn echo off so replies are clean.
        uuids = await _setup_adapter(client, elm, init=[("ATE0", "Echo off", 2.0)])
        if not uuids:
            return
        write_uuid, notify_uuid = uuids
        print("\n--- Raw commands ---\n")
        for cmd in commands:
            resp = await elm.send(client, write_uuid, cmd, 3.0)
            print(f"  > {cmd}")
            for line in (resp or "<empty>").split("\r"):
                line = line.strip()
                if line and line != cmd:
                    print(f"    {line}")
        await client.stop_notify(notify_uuid)


async def main():
    import argparse

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("address", nargs="?", help="adapter address (MAC, or CoreBluetooth UUID on macOS)")
    ap.add_argument("--sweep", action="store_true", help="brute-force DID ranges instead of the fixed PID list")
    ap.add_argument("--ecu", action="append", help=f"ECU request header(s) to sweep, default {','.join(DEFAULT_SWEEP_ECUS)}")
    ap.add_argument("--range", action="append", help=f"DID range(s) as hex LO-HI, default {','.join(DEFAULT_SWEEP_RANGES)}")
    ap.add_argument("--out", default="sweep.json", help="where to save sweep results (JSON)")
    ap.add_argument("--label", default="", help="free-text label stored in the sweep file, e.g. 'charging'")
    ap.add_argument("--compare", nargs=2, metavar=("A.json", "B.json"), help="diff two sweep files and exit")
    ap.add_argument("--raw", nargs="*", metavar="CMD",
                    help="send raw AT/ST/OBD commands and print replies; with no CMD, run the adapter sleep probe set")
    ap.add_argument("--test", action="store_true", help="self-test: run everything against a simulated adapter/car, no BLE")
    args = ap.parse_args()

    if args.test:
        sys.exit(await selftest())
    if args.compare:
        compare(*args.compare)
        return

    address = args.address
    if not address:
        candidates = await scan_adapters()
        if not candidates:
            example = (
                "E28B5EB4-7867-EA36-CBCF-6BFAABB5283E"
                if platform.system() == "Darwin"
                else "AA:BB:CC:DD:EE:FF"
            )
            print("\nTo connect to a specific device by address:")
            print(f"  python3 scripts/discover.py {example}")
            return
        if len(candidates) == 1:
            choice = 0
        else:
            try:
                choice = int(input(f"\nPick adapter [0-{len(candidates)-1}]: "))
            except (ValueError, EOFError):
                return
        address = candidates[choice].address

    if args.raw is not None:
        cmds = args.raw or [c for c, _ in SLEEP_PROBE_COMMANDS]
        if not args.raw:
            print("Adapter sleep probe:")
            for c, why in SLEEP_PROBE_COMMANDS:
                print(f"  {c:8s} {why}")
        await raw(address, cmds)
    elif args.sweep:
        ecus = [e.strip().upper() for x in (args.ecu or DEFAULT_SWEEP_ECUS) for e in x.split(",")]
        ranges = [r.strip().upper() for x in (args.range or DEFAULT_SWEEP_RANGES) for r in x.split(",")]
        await sweep(address, ecus, ranges, args.out, args.label)
    else:
        await discover(address)


if __name__ == "__main__":
    asyncio.run(main())

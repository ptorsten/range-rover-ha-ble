#!/usr/bin/env python3
"""Range Rover P550e PHEV — BLE OBD-II PID discovery script.

Scans for BLE OBD adapters, connects, initialises ELM327, and probes
JLR BECM/BCCM DIDs to find which ones the vehicle responds to.

Supports probing with the car off (e.g. while charging) by attempting
CAN bus wakeup and UDS TesterPresent keepalives.

Usage:
    pip install bleak
    python3 scripts/discover.py                  # scan + auto-pick adapter
    python3 scripts/discover.py AA:BB:CC:DD:EE   # connect to specific address
"""

import asyncio
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
    ("7E4", "224919", "BECM: SOH maximum", "soh"),
    ("7E4", "22491A", "BECM: SOH minimum", "soh"),
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
    ("7E4", "22DD04", "BECM: SOC (alt DID DD04)", "soc_alt"),
    ("7E4", "22DD05", "BECM: HV voltage (alt DID DD05)", None),
    ("7E4", "22DD06", "BECM: HV current (alt DID DD06)", None),
    ("7E4", "22DD07", "BECM: Battery temp (alt DID DD07)", None),
    ("7E4", "22DD0A", "BECM: Charging status (alt DID DD0A)", None),
    ("7E4", "22DD0B", "BECM: EV range (alt DID DD0B)", None),

    # Same alt DIDs on BCCM
    ("7E5", "22DD04", "BCCM: SOC (alt DID DD04)", "soc_alt"),
    ("7E5", "22DD0A", "BCCM: Charging status (alt DID DD0A)", None),
    ("7E5", "22DD0B", "BCCM: EV range (alt DID DD0B)", None),
]


def try_decode(raw_hex: str, decoder: str | None) -> str:
    """Attempt to decode a raw hex response into a human-readable value."""
    if not decoder or not raw_hex:
        return ""
    try:
        data = bytes.fromhex(raw_hex.replace(" ", ""))
    except ValueError:
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
    try:
        data = bytes.fromhex(raw.replace(" ", ""))
    except ValueError:
        return None
    for i in range(len(data) - 2):
        if data[i] == 0x7F:
            service = data[i + 1]
            nrc = data[i + 2]
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

    discovered: dict[str, tuple] = {}

    def _detection_callback(device, advertisement_data):
        rssi = advertisement_data.rssi if advertisement_data else None
        discovered[device.address] = (device, rssi)

    scanner = BleakScanner(detection_callback=_detection_callback)
    await scanner.start()
    await asyncio.sleep(10.0)
    await scanner.stop()

    obd_keywords = {"obd", "elm", "vgate", "vlink", "icar", "obdlink", "lelink", "car"}
    candidates = []
    all_devices = []

    for addr, (device, rssi) in discovered.items():
        name = (device.name or "").strip()
        all_devices.append((addr, name, rssi or -999))
        if any(kw in name.lower() for kw in obd_keywords):
            candidates.append((device, rssi))

    if candidates:
        print(f"Found {len(candidates)} likely OBD adapter(s):\n")
        for i, (d, rssi) in enumerate(candidates):
            rssi_str = f"RSSI: {rssi}" if rssi else ""
            print(f"  [{i}] {d.name}  ({d.address})  {rssi_str}")
    else:
        print("No obvious OBD adapters found. All BLE devices:\n")
        for addr, name, rssi in sorted(all_devices, key=lambda x: x[2], reverse=True):
            label = name if name else "(no name)"
            rssi_str = f"RSSI: {rssi}" if rssi != -999 else ""
            print(f"  {label:30s}  {addr}  {rssi_str}")
        print("\nTip: make sure the OBD adapter is plugged in.")

    return candidates


async def discover(address: str):
    """Connect to the adapter and probe all PIDs."""
    print(f"\nConnecting to {address}...\n")

    elm = ELM327BLE()

    async with BleakClient(address, timeout=20.0) as client:
        print(f"Connected: {client.is_connected}\n")

        # List services to find the right UUIDs
        services = client.services
        print("GATT services:")
        notify_uuid = None
        write_uuid = None

        for service in services:
            print(f"  Service: {service.uuid}")
            for char in service.characteristics:
                props = ", ".join(char.properties)
                print(f"    Char: {char.uuid}  [{props}]")

        # Try UUID candidates
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
            return

        # Subscribe to notifications
        await client.start_notify(notify_uuid, elm._on_notify)

        # --- ELM327 init ---
        print("\n--- ELM327 Initialisation ---\n")
        for cmd, desc, timeout in ELM_INIT:
            resp = await elm.send(client, write_uuid, cmd, timeout)
            ok = "ERROR" not in resp.upper() and "TIMEOUT" not in resp
            status = "  OK" if ok else "WARN"
            print(f"  [{status}] {cmd:10s} ({desc})")
            if resp and resp != cmd:
                for line in resp.split("\r"):
                    line = line.strip()
                    if line:
                        print(f"          -> {line}")
            await asyncio.sleep(0.3)

        # --- CAN bus / ECU wakeup ---
        print("\n--- CAN Bus Wakeup Sequence ---")
        print("  (Attempting to wake ECUs — needed when car is off/charging)\n")

        wakeup_responded = False
        last_header = None
        for header, command, label in WAKEUP_SEQUENCE:
            if header != last_header:
                await elm.send(client, write_uuid, f"ATSH{header}", 2.0)
                last_header = header
                await asyncio.sleep(0.1)

            resp = await elm.send(client, write_uuid, command, 3.0)
            resp_upper = resp.upper().replace("\r", " ").strip()
            await asyncio.sleep(0.3)

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
            await asyncio.sleep(1.0)
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
            await asyncio.sleep(0.3)

            # Periodically send TesterPresent to keep ECUs awake
            tester_present_counter += 1
            if tester_present_counter >= 8:
                tester_present_counter = 0
                saved_header = last_header
                for tp_header in ("7E4", "7E5"):
                    await elm.send(client, write_uuid, f"ATSH{tp_header}", 1.0)
                    await elm.send(client, write_uuid, "3E00", 2.0)
                    await asyncio.sleep(0.1)
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
                    decoded = try_decode(resp_upper.replace(" ", ""), decoder)
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


async def main():
    if len(sys.argv) > 1:
        address = sys.argv[1]
        await discover(address)
    else:
        candidates = await scan_adapters()
        if not candidates:
            print("\nTo connect to a specific device by address:")
            print("  python3 scripts/discover.py AA:BB:CC:DD:EE:FF")
            return

        if len(candidates) == 1:
            choice = 0
        else:
            try:
                choice = int(input(f"\nPick adapter [0-{len(candidates)-1}]: "))
            except (ValueError, EOFError):
                return

        device, _rssi = candidates[choice]
        await discover(device.address)


if __name__ == "__main__":
    asyncio.run(main())

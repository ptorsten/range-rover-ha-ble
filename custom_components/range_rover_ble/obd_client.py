"""BLE OBD-II client for Range Rover P550e PHEV.

Connects to an ELM327-compatible BLE OBD-II adapter and sends
AT/OBD commands to read vehicle data from the CAN bus.

JLR vehicles use ISO 15765-4 CAN (11-bit, 500 kbps).

Key ECU addresses:
  7E0 / 7E8  — PCM (Powertrain Control Module)
  7E2 / 7EA  — BBM (Battery Backup Module)
  7E4 / 7EC  — BECM (Battery Energy Control Module) — primary target
  7E5 / 7ED  — BCCM (Battery Charge Control Module)
  7DF         — OBD-II broadcast

Data is read using UDS Mode 22 (ReadDataByIdentifier) with JLR-specific
DIDs derived from I-Pace/JLR platform research. The P550e uses the MLA-Flex
platform — DID numbers may differ and MUST be validated on the actual vehicle.
"""

import asyncio
import logging
from collections.abc import Callable
from typing import Any

from bleak import BleakClient
from bleak.backends.device import BLEDevice
from bleak_retry_connector import BleakClientWithServiceCache, establish_connection

_LOGGER = logging.getLogger(__name__)

# ELM327 initialisation sequence
ELM_INIT_COMMANDS = [
    b"ATZ\r",       # reset
    b"ATE0\r",      # echo off
    b"ATL0\r",      # linefeeds off
    b"ATS0\r",      # spaces off
    b"ATH1\r",      # headers on (so we can see ECU source address)
    b"ATSP6\r",     # protocol: ISO 15765-4 CAN 11-bit 500kbaud
    b"ATCAF1\r",    # CAN auto formatting on
    b"ATSTFF\r",    # max timeout (needed when ECUs are slow to wake)
]

# ECU addresses to send TesterPresent (3E 00) to before reading data.
# This wakes sleeping ECUs and keeps them awake for the duration of the poll.
WAKEUP_HEADERS = ["7E4", "7E5"]

# Commands that reveal how an adapter sleeps. "ST" commands are answered only
# by STN-based adapters (OBDLink, some Vgate); a plain ELM327 clone says "?".
SLEEP_PROBE_COMMANDS: list[tuple[str, str]] = [
    ("ATI", "ELM327 identity"),
    ("AT@1", "device description"),
    ("ATRV", "12V as seen by the adapter"),
    ("STI", "STN firmware (STN chips only)"),
    ("STDI", "STN device id"),
    ("STSLCS", "STN sleep/wake config summary"),
    ("ATPPS", "ELM327 programmable parameters (PP 0E/0F = low power)"),
]

# Known BLE OBD adapter GATT layouts. If the configured UUIDs are not present
# on the connected device we fall back to the first of these that is.
UUID_CANDIDATES: list[dict[str, str]] = [
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
        "name": "Vgate vLinker / iOS-Vlink type",
        "service": "e7810a71-73ae-499d-8c15-faa9aef0c3f2",
        "notify": "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f",
        "write": "bef8d6c9-9c21-4c9e-b632-bd58c1009f9f",
    },
]


def _resolve_uuids(client: BleakClient, read_uuid: str, write_uuid: str) -> tuple[str, str]:
    """Return (notify_uuid, write_uuid) that actually exist on the device.

    Prefer the configured pair when the device has them; otherwise pick the
    first known adapter layout whose notify characteristic is present; as a
    last resort pick any characteristic that can notify and one that can be
    written to.
    """
    chars = {
        c.uuid.lower(): c
        for service in client.services
        for c in service.characteristics
    }
    if not chars:
        return read_uuid, write_uuid

    def _ok(uuid: str, prop: str) -> bool:
        c = chars.get(uuid.lower())
        return c is not None and prop in c.properties

    if _ok(read_uuid, "notify") or _ok(read_uuid, "indicate"):
        if write_uuid.lower() in chars:
            return read_uuid, write_uuid

    for cand in UUID_CANDIDATES:
        if (_ok(cand["notify"], "notify") or _ok(cand["notify"], "indicate")) and cand["write"].lower() in chars:
            _LOGGER.info(
                "Configured BLE UUIDs not found on adapter; using %s layout", cand["name"]
            )
            return cand["notify"], cand["write"]

    notify = next((u for u, c in chars.items() if "notify" in c.properties or "indicate" in c.properties), None)
    write = next((u for u, c in chars.items() if "write" in c.properties or "write-without-response" in c.properties), None)
    if notify and write:
        _LOGGER.warning(
            "Unknown BLE adapter layout; guessing notify=%s write=%s", notify, write
        )
        return notify, write
    return read_uuid, write_uuid


# OBD commands for Range Rover P550e PHEV.
# Based on JLR BECM (7E4) UDS Mode 22 DIDs from I-Pace community research.
# Standard OBD PIDs are used where applicable.
#
# IMPORTANT: These PIDs are best-effort starting points. The P550e is on the
# MLA-Flex platform while I-Pace is on D7e — specific DIDs may have shifted.
# Validate on the actual vehicle and adjust via overrides.yaml if needed.
COMMANDS: dict[str, dict[str, Any]] = {
    # --- BECM (Battery Energy Control Module) at 7E4 ---
    "state_of_charge": {
        "header": "7E4",
        "command": "224910",
        "description": "HV battery SOC (average)",
        "decoder": "soc_jlr",
    },
    "soc_min": {
        "header": "7E4",
        "command": "224911",
        "description": "HV battery SOC (minimum cell)",
        "decoder": "soc_jlr",
    },
    "hv_battery_voltage": {
        "header": "7E4",
        "command": "22490F",
        "description": "HV battery pack voltage",
        "decoder": "hv_voltage_jlr",
    },
    "hv_battery_current": {
        "header": "7E4",
        "command": "22490C",
        "description": "HV battery pack current",
        "decoder": "hv_current_jlr",
    },
    "hv_battery_temp": {
        "header": "7E4",
        "command": "224905",
        "description": "HV battery temperature",
        "decoder": "hv_temp_jlr",
    },
    "soh_avg": {
        "header": "7E4",
        "command": "224918",
        "description": "HV battery state of health (average)",
        "decoder": "soh_jlr",
    },
    "cell_voltage_max": {
        "header": "7E4",
        "command": "224903",
        "description": "Maximum cell voltage",
        "decoder": "cell_voltage_jlr",
    },
    "cell_voltage_min": {
        "header": "7E4",
        "command": "224904",
        "description": "Minimum cell voltage",
        "decoder": "cell_voltage_jlr",
    },
    # --- Standard OBD-II PIDs (7DF broadcast) ---
    "bat_12v_voltage": {
        "header": "7DF",
        "command": "0142",
        "description": "12V control module voltage",
        "decoder": "bat_12v_std",
    },
    "ambient_temp": {
        "header": "7DF",
        "command": "0146",
        "description": "Ambient air temperature",
        "decoder": "ambient_temp_std",
    },
    "speed": {
        "header": "7DF",
        "command": "010D",
        "description": "Vehicle speed",
        "decoder": "speed_std",
    },
    # --- Hybrid-specific standard PID ---
    "soc_std": {
        "header": "7DF",
        "command": "015B",
        "description": "Hybrid battery pack remaining life (standard PID)",
        "decoder": "soc_std",
    },
    # --- BCCM (Battery Charge Control Module) at 7E5 ---
    # The BCCM manages charging and may respond when the BECM doesn't
    # (e.g. car off but plugged in). Same DIDs, different ECU address.
    "bccm_soc": {
        "header": "7E5",
        "command": "224910",
        "description": "BCCM: HV battery SOC",
        "decoder": "soc_jlr",
    },
    "bccm_voltage": {
        "header": "7E5",
        "command": "22490F",
        "description": "BCCM: HV battery voltage",
        "decoder": "hv_voltage_jlr",
    },
    "bccm_current": {
        "header": "7E5",
        "command": "22490C",
        "description": "BCCM: HV battery current",
        "decoder": "hv_current_jlr",
    },
    # --- Validated on a P550e (2026-09/10) ---
    "soh_min": {
        "header": "7E4",
        "command": "224919",
        "description": "HV battery state of health (minimum)",
        "decoder": "soh_jlr",
    },
    "soh_max": {
        "header": "7E4",
        "command": "22491A",
        "description": "HV battery state of health (maximum)",
        "decoder": "soh_jlr",
    },
    "battery_plate_temp_1": {
        "header": "7E4",
        "command": "22492B",
        "description": "HV battery plate temperature 1",
        "decoder": "temp_offset_jlr",
    },
    "battery_plate_temp_2": {
        "header": "7E4",
        "command": "22492C",
        "description": "HV battery plate temperature 2",
        "decoder": "temp_offset_jlr",
    },
    # EXPERIMENTAL: 22DD06 read 0x00 with the car awake and not charging
    # (2026-09-26) and 0x04 while AC charging (2026-10-03). Treated as a
    # charging indicator until a DID sweep confirms its meaning.
    "charge_indicator": {
        "header": "7E4",
        "command": "22DD06",
        "description": "BECM DD06 (0 idle, non-zero while charging — experimental)",
        "decoder": "u8_jlr",
    },
}


def _derive_charging_status(result: dict[str, Any]) -> None:
    """Set result["charging_status"] from what the car does expose.

    No confirmed charging-status DID yet. Heuristic, in order of trust:
    1. HV current sign if the car ever answers 22490C (P550e does not).
    2. Experimental 22DD06 indicator: 0 idle, non-zero while charging.
    3. Otherwise leave unknown (binary sensor shows off).
    """
    current = result.get("hv_battery_current")
    if isinstance(current, (int, float)):
        result["charging_status"] = "charging" if current < -0.5 else "not_charging"
        return
    indicator = result.get("charge_indicator")
    if isinstance(indicator, int):
        result["charging_status"] = "charging" if indicator > 0 else "not_charging"


def _parse_frames(raw: str) -> list[bytes]:
    """Split an ELM327 response (ATH1, ATS0) into per-frame payloads.

    Each frame looks like "<3-hex-digit CAN ID><PCI byte><data>", e.g.
    "7EC05624910258C". The 3-digit ID makes the frame odd-length, so
    bytes.fromhex() on the whole string fails; strip the ID per frame and
    drop the ISO-TP PCI byte. Returns [] for ELM noise or unparseable input.
    """
    clean = raw.replace("\r", " ").replace("\n", " ").upper()
    for noise in ("SEARCHING...", "NO DATA", "NODATA", "?", "ERROR", "CAN ERROR", "BUS ERROR"):
        if noise in clean:
            return []

    payloads: list[bytes] = []
    for frame in clean.split():
        if len(frame) % 2 == 1 and len(frame) >= 5:
            body = frame[3:]          # 11-bit CAN ID
        elif len(frame) >= 10 and frame.startswith("18"):
            body = frame[8:]          # 29-bit CAN ID
        else:
            body = frame
        try:
            data = bytes.fromhex(body)
        except ValueError:
            continue
        if not data:
            continue
        pci = data[0]
        if pci >> 4 == 0:             # single frame: low nibble = length
            data = data[1 : 1 + (pci & 0x0F)]
        elif pci >> 4 == 1 and len(data) > 2:  # first frame of multi-frame
            data = data[2:]
        elif pci >> 4 == 2:           # consecutive frame
            data = data[1:]
        if data:
            payloads.append(data)
    return payloads


def _parse_hex_payload(raw: str) -> bytes | None:
    """Return the first positive-response payload, or None.

    A broadcast (7DF) can return one frame per ECU; some may be UDS negative
    responses (7F xx NRC) while another carries real data, so prefer the
    first non-7F frame. Returns None when every ECU rejected the request.
    """
    for data in _parse_frames(raw):
        if data[0] != 0x7F:
            return data
    return None


def _decode_response(raw: str, decoder: str) -> Any | None:
    """Decode an ELM327 response string into a typed value.

    Headers are on, so the raw string includes the ECU response address.
    E.g. for BECM: "7EC0362491000FF" (7EC = response from 7E4).

    Returns None if the response cannot be decoded.
    """
    data = _parse_hex_payload(raw)
    if data is None:
        return None

    # JLR BECM decoders (UDS Mode 22 responses)
    # Response format: [ECU_ID_MSB] [ECU_ID_LSB] [len] [62] [DID_H] [DID_L] [data...]
    # With ATH1, the hex includes the CAN ID prefix.

    if decoder == "soc_jlr":
        # (A*256 + B) / 100 → %
        # Find the positive response (0x62) for Mode 22
        idx = _find_uds_response(data)
        if idx is None or idx + 5 > len(data):
            return None
        a, b = data[idx + 3], data[idx + 4]
        return (a * 256 + b) / 100.0

    if decoder == "hv_voltage_jlr":
        # (A*256 + B) / 100 → V
        idx = _find_uds_response(data)
        if idx is None or idx + 5 > len(data):
            return None
        a, b = data[idx + 3], data[idx + 4]
        return (a * 256 + b) / 100.0

    if decoder == "hv_current_jlr":
        # (A*256 + B - 32768) / 40 → A (signed)
        idx = _find_uds_response(data)
        if idx is None or idx + 5 > len(data):
            return None
        a, b = data[idx + 3], data[idx + 4]
        return (a * 256 + b - 32768) / 40.0

    if decoder == "hv_temp_jlr":
        # A/2 - 40 → °C
        idx = _find_uds_response(data)
        if idx is None or idx + 4 > len(data):
            return None
        return data[idx + 3] / 2.0 - 40.0

    if decoder == "soh_jlr":
        # A / 2 → %
        idx = _find_uds_response(data)
        if idx is None or idx + 4 > len(data):
            return None
        return data[idx + 3] / 2.0

    if decoder == "temp_offset_jlr":
        # A - 40 → °C
        idx = _find_uds_response(data)
        if idx is None or idx + 4 > len(data):
            return None
        return data[idx + 3] - 40

    if decoder == "u8_jlr":
        # raw single byte
        idx = _find_uds_response(data)
        if idx is None or idx + 4 > len(data):
            return None
        return data[idx + 3]

    if decoder == "cell_voltage_jlr":
        # (A*256 + B) / 1000 → V
        idx = _find_uds_response(data)
        if idx is None or idx + 5 > len(data):
            return None
        a, b = data[idx + 3], data[idx + 4]
        return (a * 256 + b) / 1000.0

    # Standard OBD-II decoders (Mode 01/41 responses)
    if decoder == "bat_12v_std":
        # PID 42: (A*256 + B) / 1000 → V
        idx = _find_obd_response(data, 0x42)
        if idx is None or idx + 4 > len(data):
            return None
        a, b = data[idx + 2], data[idx + 3]
        return (a * 256 + b) / 1000.0

    if decoder == "ambient_temp_std":
        # PID 46: A - 40 → °C
        idx = _find_obd_response(data, 0x46)
        if idx is None or idx + 3 > len(data):
            return None
        return data[idx + 2] - 40

    if decoder == "speed_std":
        # PID 0D: A → km/h
        idx = _find_obd_response(data, 0x0D)
        if idx is None or idx + 3 > len(data):
            return None
        return data[idx + 2]

    if decoder == "soc_std":
        # PID 5B: A * 100/255 → %
        idx = _find_obd_response(data, 0x5B)
        if idx is None or idx + 3 > len(data):
            return None
        return data[idx + 2] * 100.0 / 255.0

    _LOGGER.warning("Unknown decoder: %s", decoder)
    return None


def _find_uds_response(data: bytes) -> int | None:
    """Find the index of the UDS positive response byte (0x62) in parsed data."""
    for i in range(len(data) - 1):
        if data[i] == 0x62:
            return i
    return None


def _find_obd_response(data: bytes, pid: int) -> int | None:
    """Find the index of a Mode 41 response for a given PID."""
    for i in range(len(data) - 1):
        if data[i] == 0x41 and i + 1 < len(data) and data[i + 1] == pid:
            return i
    return None


class RangeRoverBleClient:
    """BLE OBD-II client that talks ELM327 protocol over GATT."""

    def __init__(
        self,
        ble_device: BLEDevice,
        service_uuid: str | None = None,
        read_uuid: str | None = None,
        write_uuid: str | None = None,
        device_lookup: Callable[[], BLEDevice | None] | None = None,
    ) -> None:
        """Initialize with a BLE device reference.

        ``device_lookup`` is called before each connection to fetch a fresh
        BLEDevice from Home Assistant's Bluetooth manager. The device object
        carries the routing details for whichever adapter or ESPHome proxy
        currently hears the dongle, so a stale one can point at a proxy that
        no longer sees it.
        """
        self._ble_device = ble_device
        self._device_lookup = device_lookup
        self._service_uuid = service_uuid
        self._read_uuid = read_uuid
        self._write_uuid = write_uuid
        self._response_buffer = bytearray()
        self._response_event = asyncio.Event()

    async def _connect(self) -> BleakClient:
        """Connect with bleak-retry-connector (handles proxies, retries, slots)."""
        if self._device_lookup is not None:
            fresh = self._device_lookup()
            if fresh is not None:
                self._ble_device = fresh
        return await establish_connection(
            BleakClientWithServiceCache,
            self._ble_device,
            self._ble_device.name or self._ble_device.address,
            max_attempts=3,
        )

    def _notification_handler(self, _sender, data: bytearray) -> None:
        """Handle incoming BLE notifications (ELM327 responses)."""
        self._response_buffer.extend(data)
        if b">" in data:
            self._response_event.set()

    async def _send_command(
        self, client: BleakClient, command: bytes, timeout: float = 5.0
    ) -> str:
        """Send an AT/OBD command and wait for the ELM327 prompt response."""
        self._response_buffer.clear()
        self._response_event.clear()

        await client.write_gatt_char(self._write_uuid, command)

        try:
            await asyncio.wait_for(self._response_event.wait(), timeout=timeout)
        except TimeoutError:
            _LOGGER.debug("Timeout waiting for response to: %s", command)
            return ""

        response = self._response_buffer.decode("ascii", errors="replace").strip()
        response = response.replace(">", "").strip()
        return response

    async def async_get_data(self, options: dict | None = None) -> dict[str, Any]:
        """Connect to the OBD adapter, initialise ELM327, and read all PIDs.

        Returns a dict of sensor_key -> decoded_value.
        """
        options = options or {}
        read_uuid = options.get("characteristic_uuid_read", self._read_uuid)
        write_uuid = options.get("characteristic_uuid_write", self._write_uuid)

        result: dict[str, Any] = {}
        last_header = None

        try:
            client = await self._connect()
            try:
                read_uuid, write_uuid = _resolve_uuids(client, read_uuid, write_uuid)
                self._write_uuid = write_uuid
                await client.start_notify(read_uuid, self._notification_handler)

                # Initialise ELM327
                for init_cmd in ELM_INIT_COMMANDS:
                    resp = await self._send_command(client, init_cmd, timeout=3.0)
                    _LOGGER.debug("ELM init %s -> %s", init_cmd.strip(), resp)
                    if "ERROR" in resp.upper():
                        _LOGGER.warning(
                            "ELM init command failed: %s -> %s", init_cmd, resp
                        )

                # Wake ECUs with TesterPresent (3E 00) and
                # ExtendedDiagnosticSession (10 03) — needed when the car
                # is off but charging, as the CAN gateway and ECUs may be
                # in sleep mode.
                for wake_header in WAKEUP_HEADERS:
                    await self._send_command(
                        client, f"ATSH{wake_header}\r".encode(), timeout=2.0
                    )
                    resp = await self._send_command(
                        client, b"3E00\r", timeout=3.0
                    )
                    _LOGGER.debug("TesterPresent %s -> %s", wake_header, resp)
                    await self._send_command(
                        client, b"1003\r", timeout=3.0
                    )

                # Read each PID
                pid_count = 0
                for key, cmd_def in COMMANDS.items():
                    header = cmd_def["header"]
                    command = cmd_def["command"]

                    # Only set header if it changed
                    if header != last_header:
                        await self._send_command(
                            client, f"ATSH{header}\r".encode(), timeout=2.0
                        )
                        last_header = header

                    raw = await self._send_command(
                        client, f"{command}\r".encode(), timeout=5.0
                    )

                    if not raw or "NO DATA" in raw.upper() or "ERROR" in raw.upper():
                        _LOGGER.debug(
                            "No data for %s (%s): %s", key, command, raw
                        )
                        continue

                    # Take the last data line (skip echoes / multi-line noise)
                    lines = [
                        line.strip()
                        for line in raw.split("\r")
                        if line.strip()
                        and not line.strip().startswith("AT")
                        and not line.strip().startswith("SEARCHING")
                    ]
                    if not lines:
                        continue

                    value = _decode_response(lines[-1], cmd_def["decoder"])
                    if value is not None:
                        result[key] = value
                        _LOGGER.debug("Decoded %s = %s", key, value)

                    # Periodically re-send TesterPresent to keep ECUs awake
                    pid_count += 1
                    if pid_count % 6 == 0:
                        saved_header = last_header
                        for wake_header in WAKEUP_HEADERS:
                            await self._send_command(
                                client,
                                f"ATSH{wake_header}\r".encode(),
                                timeout=1.0,
                            )
                            await self._send_command(
                                client, b"3E00\r", timeout=2.0
                            )
                        if saved_header:
                            await self._send_command(
                                client,
                                f"ATSH{saved_header}\r".encode(),
                                timeout=1.0,
                            )
                            last_header = saved_header

                await client.stop_notify(read_uuid)
            finally:
                await client.disconnect()

        except Exception as err:
            _LOGGER.error("BLE OBD communication error: %s", err)
            return result

        _derive_charging_status(result)
        return result

    async def async_send_raw(
        self, commands: list[str], options: dict | None = None
    ) -> list[tuple[str, str]]:
        """Send arbitrary AT/ST/OBD commands and return (command, reply) pairs.

        Does NOT run the ELM init sequence (no ATZ), so adapter settings you
        are reading or changing are not reset first. Echo is turned off.
        """
        options = options or {}
        read_uuid = options.get("characteristic_uuid_read", self._read_uuid)
        write_uuid = options.get("characteristic_uuid_write", self._write_uuid)
        out: list[tuple[str, str]] = []

        client = await self._connect()
        try:
            read_uuid, write_uuid = _resolve_uuids(client, read_uuid, write_uuid)
            self._write_uuid = write_uuid
            await client.start_notify(read_uuid, self._notification_handler)
            await self._send_command(client, b"ATE0\r", timeout=2.0)
            for cmd in commands:
                cmd = cmd.strip()
                if not cmd:
                    continue
                resp = await self._send_command(client, f"{cmd}\r".encode(), timeout=4.0)
                resp = resp.replace("\r", "\n").strip() or "<no reply / timeout>"
                # drop echoed command line if echo was still on
                lines = [l for l in resp.splitlines() if l.strip() and l.strip() != cmd]
                out.append((cmd, "\n".join(lines) or "<empty>"))
            await client.stop_notify(read_uuid)
        finally:
            await client.disconnect()
        return out

    async def async_run_discovery(self, options: dict | None = None) -> dict[str, Any]:
        """Run a full PID discovery scan across all ECUs.

        Probes a wide set of DIDs on BECM, BCCM, PCM, and standard OBD
        broadcast. Returns structured results for display.
        """
        options = options or {}
        read_uuid = options.get("characteristic_uuid_read", self._read_uuid)
        write_uuid = options.get("characteristic_uuid_write", self._write_uuid)

        discovery_pids = [
            # Standard OBD
            ("7DF", "0100", "Supported PIDs [01-20]"),
            ("7DF", "015B", "Hybrid battery remaining life (std)"),
            ("7DF", "0142", "Control module voltage (12V)"),
            ("7DF", "0146", "Ambient air temperature"),
            ("7DF", "010D", "Vehicle speed"),
            ("7DF", "010C", "Engine RPM"),
            # BECM
            ("7E4", "224910", "BECM: SOC average"),
            ("7E4", "224911", "BECM: SOC minimum cell"),
            ("7E4", "224914", "BECM: SOC maximum cell"),
            ("7E4", "22490F", "BECM: HV battery voltage"),
            ("7E4", "22490C", "BECM: HV battery current"),
            ("7E4", "224905", "BECM: Battery temperature"),
            ("7E4", "224918", "BECM: SOH average"),
            ("7E4", "224903", "BECM: Cell voltage max"),
            ("7E4", "224904", "BECM: Cell voltage min"),
            ("7E4", "22491B", "BECM: Coolant outlet temp"),
            ("7E4", "22491C", "BECM: Coolant inlet temp"),
            # BCCM
            ("7E5", "224910", "BCCM: SOC"),
            ("7E5", "22490F", "BCCM: HV voltage"),
            ("7E5", "22490C", "BCCM: HV current"),
            ("7E5", "224905", "BCCM: Battery temp"),
            ("7E5", "224918", "BCCM: SOH"),
            # Alt DID ranges (MLA-Flex)
            ("7E4", "22DD04", "BECM: SOC (alt DD04)"),
            ("7E4", "22DD05", "BECM: HV voltage (alt DD05)"),
            ("7E4", "22DD06", "BECM: HV current (alt DD06)"),
            ("7E4", "22DD07", "BECM: Battery temp (alt DD07)"),
            ("7E4", "22DD0A", "BECM: Charging status (alt DD0A)"),
            ("7E4", "22DD0B", "BECM: EV range (alt DD0B)"),
            ("7E5", "22DD04", "BCCM: SOC (alt DD04)"),
            ("7E5", "22DD0A", "BCCM: Charging status (alt DD0A)"),
            # PCM
            ("7E0", "224910", "PCM: SOC"),
            ("7E0", "2142", "PCM: Control module voltage"),
        ]

        responded: list[dict] = []
        no_data: list[str] = []
        errors: list[dict] = []

        try:
            client = await self._connect()
            try:
                read_uuid, write_uuid = _resolve_uuids(client, read_uuid, write_uuid)
                self._write_uuid = write_uuid
                await client.start_notify(read_uuid, self._notification_handler)

                for init_cmd in ELM_INIT_COMMANDS:
                    await self._send_command(client, init_cmd, timeout=3.0)

                # Wakeup sequence
                for wake_header in WAKEUP_HEADERS:
                    await self._send_command(
                        client, f"ATSH{wake_header}\r".encode(), timeout=2.0
                    )
                    await self._send_command(client, b"3E00\r", timeout=3.0)
                    await self._send_command(client, b"1003\r", timeout=3.0)

                # Probe all PIDs
                last_header = None
                pid_count = 0
                for header, command, label in discovery_pids:
                    if header != last_header:
                        await self._send_command(
                            client, f"ATSH{header}\r".encode(), timeout=2.0
                        )
                        last_header = header

                    raw = await self._send_command(
                        client, f"{command}\r".encode(), timeout=5.0
                    )
                    raw_clean = (raw or "").upper().replace("\r", " ").strip()

                    if not raw or "TIMEOUT" in raw:
                        no_data.append(label)
                    elif "NO DATA" in raw_clean:
                        no_data.append(label)
                    elif "ERROR" in raw_clean or "?" in raw_clean:
                        errors.append({"label": label, "raw": raw_clean})
                    else:
                        nrc = _classify_nrc(raw_clean)
                        if nrc:
                            errors.append({
                                "label": label, "raw": raw_clean, "nrc": nrc,
                            })
                        else:
                            responded.append({
                                "label": label,
                                "command": command,
                                "header": header,
                                "raw": raw_clean,
                            })

                    pid_count += 1
                    if pid_count % 8 == 0:
                        saved = last_header
                        for wh in WAKEUP_HEADERS:
                            await self._send_command(
                                client, f"ATSH{wh}\r".encode(), timeout=1.0
                            )
                            await self._send_command(
                                client, b"3E00\r", timeout=2.0
                            )
                        if saved:
                            await self._send_command(
                                client, f"ATSH{saved}\r".encode(), timeout=1.0
                            )
                            last_header = saved

                await client.stop_notify(read_uuid)
            finally:
                await client.disconnect()

        except Exception as err:
            _LOGGER.error("Discovery scan error: %s", err)
            errors.append({"label": "CONNECTION", "raw": str(err)})

        return {
            "responded": responded,
            "no_data": no_data,
            "errors": errors,
        }


def _classify_nrc(raw: str) -> str | None:
    """Name the UDS negative response code if *every* ECU answered 7F."""
    frames = _parse_frames(raw)
    if not frames or any(data[0] != 0x7F for data in frames):
        return None
    nrc_names = {
        0x10: "generalReject",
        0x11: "serviceNotSupported",
        0x12: "subFunctionNotSupported",
        0x13: "incorrectMessageLength",
        0x22: "conditionsNotCorrect",
        0x31: "requestOutOfRange",
        0x33: "securityAccessDenied",
        0x78: "requestCorrectlyReceivedResponsePending",
        0x7E: "subFunctionNotSupportedInActiveSession",
        0x7F: "serviceNotSupportedInActiveSession",
    }
    for data in frames:
        if len(data) >= 3:
            return nrc_names.get(data[2], f"0x{data[2]:02X}")
    return None

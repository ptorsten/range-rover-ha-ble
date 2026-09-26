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
from typing import Any

from bleak import BleakClient
from bleak.backends.device import BLEDevice

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
}


def _parse_hex_payload(raw: str) -> bytes | None:
    """Extract the hex data payload from an ELM327 response.

    Handles multi-line responses and strips header bytes.
    Returns None on parse failure.
    """
    clean = raw.replace(" ", "").replace("\r", "").replace("\n", "")
    # Remove common ELM noise
    for noise in ("SEARCHING...", "NODATA", "?", "ERROR", "CANERROR", "BUSERROR"):
        if noise in clean.upper():
            return None
    try:
        return bytes.fromhex(clean)
    except ValueError:
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
    ) -> None:
        """Initialize with a BLE device reference."""
        self._ble_device = ble_device
        self._service_uuid = service_uuid
        self._read_uuid = read_uuid
        self._write_uuid = write_uuid
        self._response_buffer = bytearray()
        self._response_event = asyncio.Event()

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
            async with BleakClient(self._ble_device) as client:
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

        except Exception as err:
            _LOGGER.error("BLE OBD communication error: %s", err)
            return result

        return result

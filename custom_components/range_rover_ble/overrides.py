"""Load user-defined OBD command overrides for the Range Rover BLE integration."""

import importlib.util
import logging
import struct
from pathlib import Path
from typing import Any, Callable

import yaml
from homeassistant.components.sensor import (
    SensorDeviceClass,
    SensorEntityDescription,
    SensorStateClass,
)
from homeassistant.core import HomeAssistant

from .const import DECODERS_MODULE_FILE, OVERRIDES_FILE
from .obd_client import COMMANDS as BUILTIN_COMMANDS

_LOGGER = logging.getLogger(__name__)

_DEVICE_CLASSES: dict[str, SensorDeviceClass] = {
    "battery": SensorDeviceClass.BATTERY,
    "current": SensorDeviceClass.CURRENT,
    "distance": SensorDeviceClass.DISTANCE,
    "energy": SensorDeviceClass.ENERGY,
    "enum": SensorDeviceClass.ENUM,
    "power": SensorDeviceClass.POWER,
    "pressure": SensorDeviceClass.PRESSURE,
    "speed": SensorDeviceClass.SPEED,
    "temperature": SensorDeviceClass.TEMPERATURE,
    "voltage": SensorDeviceClass.VOLTAGE,
}

_STATE_CLASSES: dict[str, SensorStateClass] = {
    "measurement": SensorStateClass.MEASUREMENT,
    "total": SensorStateClass.TOTAL,
    "total_increasing": SensorStateClass.TOTAL_INCREASING,
}


def load_overrides(
    hass: HomeAssistant, address: str
) -> tuple[dict[str, dict], dict[str, SensorEntityDescription], set[str]]:
    """Return (extra_commands, extra_sensor_descriptions, disabled_commands).

    Reads overrides.yaml and optionally decoders.py from the integration config directory.
    Safe to call from an executor thread (synchronous I/O only).
    """
    overrides_path = Path(hass.config.config_dir) / OVERRIDES_FILE
    decoders_path = Path(hass.config.config_dir) / DECODERS_MODULE_FILE

    python_module = None
    if decoders_path.exists():
        python_module = _load_python_module(decoders_path)

    if not overrides_path.exists():
        return {}, {}, set()

    try:
        with open(overrides_path) as f:
            config = yaml.safe_load(f)
    except Exception as err:
        _LOGGER.error("Failed to load %s: %s", overrides_path, err)
        return {}, {}, set()

    if not config:
        return {}, {}, set()

    command_entries: dict[str, Any] = {}
    if "_all_" in config:
        command_entries.update(((config["_all_"] or {}).get("commands", {})))
    address_upper = address.upper()
    if address_upper in config:
        command_entries.update(((config[address_upper] or {}).get("commands", {})))

    extra_commands: dict[str, dict] = {}
    extra_sensor_descriptions: dict[str, SensorEntityDescription] = {}
    disabled_commands: set[str] = set()

    for key, entry in command_entries.items():
        entry = entry or {}

        if not entry.get("enabled", True):
            disabled_commands.add(key)
            continue

        extra_commands[key] = entry

        if "sensor" in entry and key not in BUILTIN_COMMANDS:
            try:
                extra_sensor_descriptions.update(_build_sensor_descriptions(key, entry))
            except Exception as err:
                _LOGGER.error("Invalid sensor definition for command '%s': %s", key, err)

    return extra_commands, extra_sensor_descriptions, disabled_commands


def _load_python_module(path: Path):
    """Load a Python module from a file path."""
    try:
        spec = importlib.util.spec_from_file_location("range_rover_ble_decoders", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _LOGGER.info("Loaded custom decoders from %s", path)
        return module
    except Exception as err:
        _LOGGER.error("Failed to load custom decoders from %s: %s", path, err)
        return None


def _build_sensor_descriptions(
    key: str, entry: dict
) -> dict[str, SensorEntityDescription]:
    """Build SensorEntityDescription(s) from the sensor: block of an override entry."""
    sensor_block = entry["sensor"]

    if isinstance(sensor_block, list):
        return {
            field["key"]: _sensor_desc_from_block(field["key"], field)
            for field in sensor_block
        }

    return {key: _sensor_desc_from_block(key, sensor_block)}


def _sensor_desc_from_block(key: str, block: dict) -> SensorEntityDescription:
    """Build a SensorEntityDescription from a sensor config block."""
    device_class_str = block.get("device_class")
    device_class = _DEVICE_CLASSES.get(device_class_str) if device_class_str else None

    state_class_str = block.get("state_class")
    state_class = _STATE_CLASSES.get(state_class_str) if state_class_str else None

    return SensorEntityDescription(
        key=key,
        name=block.get("name", key),
        native_unit_of_measurement=block.get("unit"),
        device_class=device_class,
        state_class=state_class,
        suggested_display_precision=block.get("precision"),
        icon=block.get("icon"),
    )

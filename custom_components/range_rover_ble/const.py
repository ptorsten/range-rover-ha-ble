"""Constants for Range Rover BLE."""

from homeassistant.const import Platform

NAME = "Range Rover BLE"
DOMAIN = "range_rover_ble"
DOMAIN_DATA = f"{DOMAIN}_data"
VERSION = "0.1.0"

ISSUE_URL = "https://github.com/ptorsten/range-rover-ha-ble/issues"

PLATFORMS: list[Platform] = [Platform.BINARY_SENSOR, Platform.BUTTON, Platform.SENSOR]

CONF_ENABLED = "enabled"
CONF_SERVICE_UUID = "service_uuid"
CONF_CHARACTERISTIC_UUID_READ = "characteristic_uuid_read"
CONF_CHARACTERISTIC_UUID_WRITE = "characteristic_uuid_write"

DEFAULT_NAME = DOMAIN

# Common ELM327 BLE dongle UUIDs — override in config if your adapter differs
DEFAULT_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
DEFAULT_CHARACTERISTIC_UUID_READ = "0000ffe1-0000-1000-8000-00805f9b34fb"
DEFAULT_CHARACTERISTIC_UUID_WRITE = "0000ffe1-0000-1000-8000-00805f9b34fb"

OVERRIDES_FILE = "custom_components/range_rover_ble/overrides.yaml"
DECODERS_MODULE_FILE = "custom_components/range_rover_ble/decoders.py"

STARTUP_MESSAGE = f"""
-------------------------------------------------------------------
{NAME}
Version: {VERSION}
This is a custom integration!
If you have any issues with this you need to open an issue here:
{ISSUE_URL}
-------------------------------------------------------------------
"""

"""Custom integration to read Range Rover P550e PHEV data via BLE OBD-II.

For more details about this integration, please refer to
https://github.com/ptorsten/range-rover-ha-ble

Derived from "Nissan Leaf OBD BLE" by @pbutterworth
(https://github.com/pbutterworth/nissan-leaf-obd-ble, MIT). See LICENSE.
"""

import logging

from bleak_retry_connector import get_device

from homeassistant.components import bluetooth
from homeassistant.config_entries import ConfigEntry
import voluptuous as vol

from homeassistant.components.persistent_notification import async_create
from homeassistant.const import CONF_ADDRESS
from homeassistant.core import HomeAssistant, ServiceCall, ServiceResponse, SupportsResponse, callback
from homeassistant.exceptions import ConfigEntryNotReady
from homeassistant.helpers.typing import ConfigType

from .const import (
    DEFAULT_CHARACTERISTIC_UUID_READ,
    DEFAULT_CHARACTERISTIC_UUID_WRITE,
    DEFAULT_SERVICE_UUID,
    DOMAIN,
    PLATFORMS,
    STARTUP_MESSAGE,
)
from .coordinator import RangeRoverBleDataUpdateCoordinator
from .obd_client import RangeRoverBleClient
from .sweep import run_sweep

_LOGGER: logging.Logger = logging.getLogger(__package__)


SERVICE_SEND_RAW = "send_raw_commands"
SERVICE_SEND_RAW_SCHEMA = vol.Schema(
    {
        vol.Required("commands"): vol.All(vol.Coerce(list), [str]),
        vol.Optional("notify", default=True): bool,
    }
)


async def async_setup(hass: HomeAssistant, config: ConfigType):
    """Set up this integration using YAML is not supported."""

    async def _handle_send_raw(call: ServiceCall) -> ServiceResponse:
        coordinators = list(hass.data.get(DOMAIN, {}).values())
        if not coordinators:
            raise vol.Invalid("No Range Rover BLE adapter is configured")
        coordinator = coordinators[0]
        replies = await coordinator.client.async_send_raw(
            call.data["commands"], coordinator.options
        )
        if call.data.get("notify", True):
            body = "\n".join(f"**`{c}`**\n```\n{r}\n```" for c, r in replies)
            async_create(
                hass, body or "no replies",
                title="Range Rover BLE raw commands",
                notification_id="range_rover_ble_raw",
            )
        return {"replies": [{"command": c, "reply": r} for c, r in replies]}

    async def _handle_sweep(call: ServiceCall) -> None:
        items = list(hass.data.get(DOMAIN, {}).items())
        if not items:
            raise vol.Invalid("No Range Rover BLE adapter is configured")
        entry_id, coordinator = items[0]
        hass.async_create_background_task(
            run_sweep(
                hass, coordinator, entry_id,
                ecus=call.data.get("ecus"),
                ranges=call.data.get("ranges"),
                label=call.data.get("label"),
            ),
            "range_rover_ble DID sweep (service)",
        )

    if not hass.services.has_service(DOMAIN, "sweep_dids"):
        hass.services.async_register(
            DOMAIN,
            "sweep_dids",
            _handle_sweep,
            schema=vol.Schema(
                {
                    vol.Optional("ecus"): vol.All(vol.Coerce(list), [str]),
                    vol.Optional("ranges"): vol.All(vol.Coerce(list), [str]),
                    vol.Optional("label"): str,
                }
            ),
        )

    if not hass.services.has_service(DOMAIN, SERVICE_SEND_RAW):
        hass.services.async_register(
            DOMAIN,
            SERVICE_SEND_RAW,
            _handle_send_raw,
            schema=SERVICE_SEND_RAW_SCHEMA,
            supports_response=SupportsResponse.OPTIONAL,
        )
    return True


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry):
    """Set up this integration using UI."""
    if hass.data.get(DOMAIN) is None:
        hass.data.setdefault(DOMAIN, {})
        _LOGGER.info(STARTUP_MESSAGE)

    address: str = entry.data[CONF_ADDRESS]
    ble_device = bluetooth.async_ble_device_from_address(
        hass, address.upper(), True
    ) or await get_device(address)
    if not ble_device:
        raise ConfigEntryNotReady(
            f"Could not find BLE OBD device with address {address}"
        )

    opts = entry.options or {}
    client = RangeRoverBleClient(
        ble_device,
        service_uuid=opts.get("service_uuid", DEFAULT_SERVICE_UUID),
        read_uuid=opts.get("characteristic_uuid_read", DEFAULT_CHARACTERISTIC_UUID_READ),
        write_uuid=opts.get("characteristic_uuid_write", DEFAULT_CHARACTERISTIC_UUID_WRITE),
        device_lookup=lambda: bluetooth.async_ble_device_from_address(
            hass, address.upper(), True
        ),
    )

    coordinator = RangeRoverBleDataUpdateCoordinator(
        hass,
        address=address,
        client=client,
        options=opts,
    )

    hass.data[DOMAIN][entry.entry_id] = coordinator

    await coordinator.async_config_entry_first_refresh()
    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)

    @callback
    def _async_specific_device_found(
        service_info: bluetooth.BluetoothServiceInfoBleak,
        change: bluetooth.BluetoothChange,
    ) -> None:
        """Handle re-discovery of the device."""
        _LOGGER.debug("BLE device re-discovered: %s - %s", service_info, change)
        hass.async_create_task(coordinator.async_request_refresh())

    entry.async_on_unload(
        bluetooth.async_register_callback(
            hass,
            _async_specific_device_found,
            {"address": address},
            bluetooth.BluetoothScanningMode.ACTIVE,
        )
    )

    async def update_options_listener(hass: HomeAssistant | None, entry: ConfigEntry):
        """Handle options update."""
        coordinator.options = entry.options

    entry.async_on_unload(entry.add_update_listener(update_options_listener))

    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Handle removal of an entry."""
    unloaded = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    hass.data.pop(DOMAIN)
    return unloaded


async def async_reload_entry(hass: HomeAssistant, entry: ConfigEntry) -> None:
    """Reload config entry."""
    await async_unload_entry(hass, entry)
    await async_setup_entry(hass, entry)

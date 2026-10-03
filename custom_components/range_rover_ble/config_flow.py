"""Config flow for Range Rover BLE."""

from typing import Any

try:
    from bluetooth_data_tools import human_readable_name
except ImportError:
    def human_readable_name(_manufacturer: str | None, name: str | None, address: str):
        """Fallback if bluetooth_data_tools is unavailable."""
        return name or address

import voluptuous as vol

from homeassistant import config_entries
from homeassistant.components.bluetooth import (
    BluetoothServiceInfoBleak,
    async_discovered_service_info,
)
from homeassistant.const import CONF_ADDRESS
from homeassistant.core import callback
from homeassistant.data_entry_flow import FlowResult

from .const import (
    CONF_CHARACTERISTIC_UUID_READ,
    CONF_CHARACTERISTIC_UUID_WRITE,
    CONF_SERVICE_UUID,
    DEFAULT_CHARACTERISTIC_UUID_READ,
    DEFAULT_CHARACTERISTIC_UUID_WRITE,
    DEFAULT_SERVICE_UUID,
    DOMAIN,
)

# Substrings (matched case-insensitively) that mark a BLE device as a likely
# OBD-II adapter. Vgate's current adapters advertise as "vLinker MC", "vLinker
# FS", "vLinker MC-Android", "IOS-Vlink", ...; clones use "OBDII", "OBDBLE",
# "ELM327", "V-LINK", "Carista", "OBDLink CX", "LELink", "iCar".
OBD_NAME_KEYWORDS = (
    "obd", "elm", "vlink", "v-link", "vgate", "icar", "carista", "lelink", "kiwi",
)


def _looks_like_obd_adapter(name: str | None) -> bool:
    """Return True if the advertised name looks like an OBD-II adapter."""
    lowered = (name or "").lower()
    return any(kw in lowered for kw in OBD_NAME_KEYWORDS)


class ConfigFlow(config_entries.ConfigFlow, domain=DOMAIN):
    """Config flow handler."""

    VERSION = 1
    CONNECTION_CLASS = config_entries.CONN_CLASS_LOCAL_POLL

    def __init__(self) -> None:
        """Initialize."""
        self._errors = {}
        self._discovery_info: BluetoothServiceInfoBleak | None = None
        self._discovered_devices: dict[str, BluetoothServiceInfoBleak] = {}
        self._selected_device: BluetoothServiceInfoBleak | None = None
        self._show_all_hint = False

    @staticmethod
    @callback
    def async_get_options_flow(
        config_entry: config_entries.ConfigEntry,
    ) -> config_entries.OptionsFlow:
        """Return the options flow."""
        return RangeRoverBleOptionsFlowHandler()

    async def async_step_bluetooth(
        self, discovery_info: BluetoothServiceInfoBleak
    ) -> FlowResult:
        """Handle the bluetooth discovery step."""
        await self.async_set_unique_id(discovery_info.address)
        self._abort_if_unique_id_configured()
        self._discovery_info = discovery_info
        self.context["title_placeholders"] = {
            "name": human_readable_name(
                None, discovery_info.name, discovery_info.address
            )
        }
        return await self.async_step_user()

    async def async_step_user(self, user_input: dict | None = None) -> FlowResult:
        """Handle the user step to pick discovered device."""
        errors: dict[str, str] = {}

        if user_input is not None:
            address = user_input[CONF_ADDRESS]
            discovery_info = self._discovered_devices[address]
            await self.async_set_unique_id(
                discovery_info.address, raise_on_progress=False
            )
            self._abort_if_unique_id_configured()
            self._selected_device = discovery_info
            return await self.async_step_configure()

        if discovery := self._discovery_info:
            self._discovered_devices[discovery.address] = discovery
        else:
            current_addresses = self._async_current_ids()
            likely: dict[str, BluetoothServiceInfoBleak] = {}
            others: dict[str, BluetoothServiceInfoBleak] = {}
            for discovery in async_discovered_service_info(self.hass, connectable=True):
                if (
                    discovery.address in current_addresses
                    or discovery.address in self._discovered_devices
                ):
                    continue
                if _looks_like_obd_adapter(discovery.name):
                    likely[discovery.address] = discovery
                elif discovery.name and discovery.name != discovery.address:
                    others[discovery.address] = discovery
            if likely:
                self._discovered_devices.update(likely)
            else:
                # Nothing advertised an OBD-like name. Rather than abort, let
                # the user pick from every named, connectable device in range,
                # strongest signal first. Adapters with odd names (or with no
                # service UUID in their advertisement) are still reachable.
                self._discovered_devices.update(
                    dict(
                        sorted(
                            others.items(),
                            key=lambda kv: kv[1].rssi or -999,
                            reverse=True,
                        )
                    )
                )
                self._show_all_hint = True

        if not self._discovered_devices:
            # Not a single named, connectable BLE device is visible to any
            # Bluetooth source: the adapter is asleep, out of range, or HA has
            # no Bluetooth adapter/proxy near the car.
            return self.async_abort(reason="no_unconfigured_devices")

        data_schema = vol.Schema(
            {
                vol.Required(CONF_ADDRESS): vol.In(
                    {
                        service_info.address: (
                            f"{service_info.name} ({service_info.address})"
                            + (f"  RSSI {service_info.rssi}" if service_info.rssi is not None else "")
                        )
                        for service_info in self._discovered_devices.values()
                    }
                ),
            }
        )
        return self.async_show_form(
            step_id="user",
            data_schema=data_schema,
            errors=errors,
            description_placeholders={
                "hint": (
                    "No device advertised an OBD-adapter name, so every named "
                    "Bluetooth device in range is listed. Pick your adapter "
                    "(for example vLinker, OBDII, V-LINK)."
                    if self._show_all_hint
                    else "Select your OBD-II Bluetooth adapter."
                )
            },
        )

    async def async_step_configure(
        self, user_input: dict | None = None
    ) -> FlowResult:
        """Handle UUID configuration step."""
        if user_input is not None:
            return self.async_create_entry(
                title=self._selected_device.name,
                data={CONF_ADDRESS: self._selected_device.address},
                options=user_input,
            )
        return self.async_show_form(
            step_id="configure",
            data_schema=vol.Schema(
                {
                    vol.Optional(
                        CONF_SERVICE_UUID,
                        default=DEFAULT_SERVICE_UUID,
                    ): str,
                    vol.Optional(
                        CONF_CHARACTERISTIC_UUID_READ,
                        default=DEFAULT_CHARACTERISTIC_UUID_READ,
                    ): str,
                    vol.Optional(
                        CONF_CHARACTERISTIC_UUID_WRITE,
                        default=DEFAULT_CHARACTERISTIC_UUID_WRITE,
                    ): str,
                }
            ),
        )


class RangeRoverBleOptionsFlowHandler(config_entries.OptionsFlow):
    """Config flow options handler for range_rover_ble."""

    def __init__(self) -> None:
        """Initialize options flow."""
        self.options: dict = {}

    async def async_step_init(
        self, user_input: dict[str, Any] | None = None
    ) -> FlowResult:
        """Manage the options."""
        if not self.options:
            self.options = dict(self.config_entry.options)

        if user_input is not None:
            self.options.update(user_input)
            return await self._update_options()

        return self.async_show_form(
            step_id="init",
            data_schema=vol.Schema(
                {
                    vol.Required(
                        "cache_values", default=self.options.get("cache_values", False)
                    ): bool,
                    vol.Required(
                        "fast_poll", default=self.options.get("fast_poll", 60)
                    ): int,
                    vol.Required(
                        "slow_poll", default=self.options.get("slow_poll", 900)
                    ): int,
                    vol.Required(
                        "xs_poll", default=self.options.get("xs_poll", 3600)
                    ): int,
                    vol.Required(
                        "low_12v_threshold",
                        default=self.options.get("low_12v_threshold", 12.2),
                    ): vol.Coerce(float),
                    vol.Required(
                        "low_12v_poll", default=self.options.get("low_12v_poll", 7200)
                    ): int,
                    vol.Optional(
                        CONF_SERVICE_UUID,
                        default=self.options.get(CONF_SERVICE_UUID)
                        or DEFAULT_SERVICE_UUID,
                    ): str,
                    vol.Optional(
                        CONF_CHARACTERISTIC_UUID_READ,
                        default=self.options.get(CONF_CHARACTERISTIC_UUID_READ)
                        or DEFAULT_CHARACTERISTIC_UUID_READ,
                    ): str,
                    vol.Optional(
                        CONF_CHARACTERISTIC_UUID_WRITE,
                        default=self.options.get(CONF_CHARACTERISTIC_UUID_WRITE)
                        or DEFAULT_CHARACTERISTIC_UUID_WRITE,
                    ): str,
                }
            ),
        )

    async def _update_options(self):
        """Update config entry options."""
        return self.async_create_entry(
            title=self.config_entry.data.get(CONF_ADDRESS), data=self.options
        )

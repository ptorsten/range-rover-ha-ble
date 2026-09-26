"""Coordinator for Range Rover BLE."""

import asyncio
from datetime import timedelta
import logging
from typing import Any

from homeassistant.components.bluetooth.api import async_address_present
from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed

from .const import DOMAIN
from .obd_client import RangeRoverBleClient

_LOGGER = logging.getLogger(__name__)

FAST_POLL_INTERVAL = timedelta(seconds=10)
SLOW_POLL_INTERVAL = timedelta(minutes=5)
ULTRA_SLOW_POLL_INTERVAL = timedelta(hours=1)

DEFAULT_FAST_POLL = 10
DEFAULT_SLOW_POLL = 300
DEFAULT_XS_POLL = 3600
DEFAULT_CACHE_VALUES = True
DEFAULT_FETCH_TIMEOUT = 90


class RangeRoverBleDataUpdateCoordinator(DataUpdateCoordinator):
    """Class to manage fetching data from the BLE OBD adapter."""

    def __init__(
        self,
        hass: HomeAssistant,
        address: str,
        client: RangeRoverBleClient,
        options,
    ) -> None:
        """Initialize."""
        super().__init__(
            hass,
            _LOGGER,
            name=DOMAIN,
            update_interval=FAST_POLL_INTERVAL,
            always_update=True,
        )
        self._address = address
        self.client = client
        self._cache_data: dict[str, Any] = {}
        self.options = options

    async def _async_update_data(self) -> dict[str, Any]:
        """Update data via BLE OBD adapter."""
        _LOGGER.debug("Checking if BLE OBD device is available")
        available = async_address_present(self.hass, self._address, connectable=True)
        if not available:
            _LOGGER.debug("Car out of range, switching to ultra slow polling")
            self.update_interval = timedelta(seconds=self._xs_poll_interval)
            if self.options.get("cache_values", False):
                return self._cache_data
            return {}

        try:
            new_data = await asyncio.wait_for(
                self.client.async_get_data(self.options),
                timeout=self._fetch_timeout,
            )
            if new_data is None:
                raise UpdateFailed("Failed to connect to OBD device")
            if len(new_data) == 0:
                self.update_interval = timedelta(seconds=self._slow_poll_interval)
                _LOGGER.debug(
                    "Car is probably off, switching to slow polling: interval = %s",
                    self.update_interval,
                )
            else:
                self.update_interval = timedelta(seconds=self._fast_poll_interval)
                _LOGGER.debug(
                    "Car is on, polling: interval = %s",
                    self.update_interval,
                )
        except TimeoutError as err:
            raise UpdateFailed(
                f"BLE fetch timed out after {self._fetch_timeout}s"
            ) from err
        except Exception as err:
            raise UpdateFailed(f"Unable to fetch data: {err}") from err
        else:
            if self.options.get("cache_values", False):
                self._cache_data.update(new_data)
                return self._cache_data
            return new_data

    @property
    def options(self):
        """User configuration options."""
        return self._options

    @options.setter
    def options(self, options):
        """Set the configuration options."""
        self._options = options
        self._fast_poll_interval = options.get("fast_poll", DEFAULT_FAST_POLL)
        self._slow_poll_interval = options.get("slow_poll", DEFAULT_SLOW_POLL)
        self._xs_poll_interval = options.get("xs_poll", DEFAULT_XS_POLL)
        self._cache_values = options.get("cache_values", DEFAULT_CACHE_VALUES)
        self._fetch_timeout = float(
            options.get("fetch_timeout", DEFAULT_FETCH_TIMEOUT)
        )

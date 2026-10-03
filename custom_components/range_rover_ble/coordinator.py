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

# How many consecutive "adapter not advertising" checks before we drop from the
# slow interval to the extra-slow one. A BLE peripheral stops advertising while
# another central (a phone, a laptop running discover.py) is connected to it,
# so a short absence does not mean the car has left.
ABSENT_CHECKS_BEFORE_XS = 6
# Consecutive failed polls (connect timeouts etc.) before backing off to slow.
FAILED_POLLS_BEFORE_SLOW = 3


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
        self._absent_checks = 0
        self._failed_polls = 0
        self.options = options

    def _set_interval(self, seconds: float, why: str) -> None:
        new = timedelta(seconds=seconds)
        if self.update_interval != new:
            _LOGGER.debug("%s: polling every %s", why, new)
            self.update_interval = new

    async def _async_update_data(self) -> dict[str, Any]:
        """Update data via BLE OBD adapter."""
        available = async_address_present(self.hass, self._address, connectable=True)
        if not available:
            self._absent_checks += 1
            if self._absent_checks >= ABSENT_CHECKS_BEFORE_XS:
                self._set_interval(self._xs_poll_interval, "Adapter not seen for a while")
            else:
                self._set_interval(
                    self._slow_poll_interval,
                    "Adapter not advertising (out of range, asleep, or another device is connected)",
                )
            return self._cache_data if self._cache_values else {}
        self._absent_checks = 0

        try:
            new_data = await asyncio.wait_for(
                self.client.async_get_data(self.options),
                timeout=self._fetch_timeout,
            )
        except TimeoutError as err:
            self._note_failure()
            raise UpdateFailed(
                f"BLE fetch timed out after {self._fetch_timeout}s"
            ) from err
        except Exception as err:
            self._note_failure()
            raise UpdateFailed(f"Unable to fetch data: {err}") from err

        if new_data is None:
            self._note_failure()
            raise UpdateFailed("Failed to connect to OBD device")

        self._failed_polls = 0
        if len(new_data) == 0:
            self._set_interval(self._slow_poll_interval, "Car is probably off")
        else:
            self._set_interval(self._fast_poll_interval, "Car is on")

        if self._cache_values:
            self._cache_data.update(new_data)
            return self._cache_data
        return new_data

    def _note_failure(self) -> None:
        self._failed_polls += 1
        if self._failed_polls >= FAILED_POLLS_BEFORE_SLOW:
            self._set_interval(
                self._slow_poll_interval,
                f"{self._failed_polls} consecutive failed polls",
            )

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
        # Options changed at runtime: leave any backoff and poll again soon
        # with the new intervals instead of waiting out the old timer.
        if getattr(self, "_listeners", None):
            self._absent_checks = 0
            self._failed_polls = 0
            self.update_interval = timedelta(seconds=self._fast_poll_interval)

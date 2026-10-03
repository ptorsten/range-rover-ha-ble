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

# Poll intervals (seconds) per car state. Every active poll sends CAN requests,
# and TesterPresent keep-alives stop a parked car's modules from sleeping, so
# the idle/asleep intervals are deliberately long.
DEFAULT_FAST_POLL = 60          # charging or driving: modules are awake anyway
DEFAULT_SLOW_POLL = 900         # awake but idle: passive polls only
DEFAULT_XS_POLL = 3600          # asleep or out of range
DEFAULT_LOW_12V_THRESHOLD = 12.2  # volts; below this on a parked car, back off hard
DEFAULT_LOW_12V_POLL = 7200     # seconds between polls while 12V is low (0 = pause)
DEFAULT_CACHE_VALUES = True
DEFAULT_FETCH_TIMEOUT = 90

# 12V above this means the DC-DC converter is running: car on or charging.
DCDC_ACTIVE_VOLTS = 13.2

MODE_CHARGING = "charging"
MODE_DRIVING = "driving"
MODE_IDLE = "idle"
MODE_ASLEEP = "asleep"
MODE_OUT_OF_RANGE = "out_of_range"
MODE_LOW_12V = "low_12v"
MODE_UNKNOWN = "unknown"


def determine_mode(data: dict[str, Any], low_12v_threshold: float) -> str:
    """Classify what the car is doing from one poll's decoded data."""
    if not data:
        return MODE_ASLEEP
    v12 = data.get("bat_12v_voltage")
    if data.get("charging_status") == "charging":
        return MODE_CHARGING
    speed = data.get("speed") or 0
    if speed > 0 or (isinstance(v12, (int, float)) and v12 >= DCDC_ACTIVE_VOLTS):
        return MODE_DRIVING
    if isinstance(v12, (int, float)) and v12 < low_12v_threshold:
        return MODE_LOW_12V
    return MODE_IDLE

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
        self.poll_mode: str = MODE_UNKNOWN
        self._force_wake_next = True   # first poll and manual refreshes wake the modules
        self.options = options

    def _set_interval(self, seconds: float, why: str) -> None:
        new = timedelta(seconds=seconds)
        if self.update_interval != new:
            _LOGGER.debug("%s: polling every %s", why, new)
            self.update_interval = new

    async def async_request_refresh(self) -> None:
        """Manual refresh (Refresh button, re-discovery): allowed to wake modules."""
        self._force_wake_next = True
        await super().async_request_refresh()

    async def _async_update_data(self) -> dict[str, Any]:
        """Update data via BLE OBD adapter."""
        available = async_address_present(self.hass, self._address, connectable=True)
        if not available:
            self._absent_checks += 1
            self.poll_mode = MODE_OUT_OF_RANGE
            if self._absent_checks >= ABSENT_CHECKS_BEFORE_XS:
                self._set_interval(self._xs_poll_interval, "Adapter not seen for a while")
            else:
                self._set_interval(
                    self._slow_poll_interval,
                    "Adapter not advertising (out of range, asleep, or another device is connected)",
                )
            return self._cache_data if self._cache_values else {}
        self._absent_checks = 0

        # Only wake the car's modules when we already know they are awake
        # (charging/driving) or the user asked for a refresh. Everything else
        # is a passive poll that leaves a sleeping car alone.
        wake = self._force_wake_next or self.poll_mode in (MODE_CHARGING, MODE_DRIVING)
        self._force_wake_next = False

        try:
            new_data = await asyncio.wait_for(
                self.client.async_get_data(self.options, wake_ecus=wake),
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
        self.poll_mode = determine_mode(new_data, self._low_12v_threshold)
        if self.poll_mode in (MODE_CHARGING, MODE_DRIVING):
            self._set_interval(self._fast_poll_interval, f"Car is {self.poll_mode}")
        elif self.poll_mode == MODE_IDLE:
            self._set_interval(self._slow_poll_interval, "Car awake but idle; passive polling")
        elif self.poll_mode == MODE_LOW_12V:
            if self._low_12v_poll <= 0:
                _LOGGER.warning(
                    "12V battery at %.2f V is below %.2f V; pausing polling until a manual refresh",
                    new_data.get("bat_12v_voltage", 0), self._low_12v_threshold,
                )
                self.update_interval = None
            else:
                _LOGGER.warning(
                    "12V battery at %.2f V is below %.2f V; polling only every %ss",
                    new_data.get("bat_12v_voltage", 0), self._low_12v_threshold, self._low_12v_poll,
                )
                self._set_interval(self._low_12v_poll, "Low 12V battery")
        else:  # asleep
            self._set_interval(self._xs_poll_interval, "Car asleep")

        if self._cache_values:
            self._cache_data.update(new_data)
            if new_data:
                self._cache_data["poll_mode"] = self.poll_mode
            return self._cache_data
        new_data["poll_mode"] = self.poll_mode
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
        self._low_12v_threshold = float(options.get("low_12v_threshold", DEFAULT_LOW_12V_THRESHOLD))
        self._low_12v_poll = int(options.get("low_12v_poll", DEFAULT_LOW_12V_POLL))
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

"""Coordinator for Range Rover BLE."""

import asyncio
from collections import deque
from datetime import datetime, timedelta, timezone
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
# Raw-SOC window the dashboard maps to 0–100 %. Confirmed on a P550e at raw
# 57.3 / 57.4 / 60.2 % -> shown 49 / 50 / 53 %, and raw ~96 % when full.
DEFAULT_SOC_RAW_EMPTY = 20.0
DEFAULT_SOC_RAW_FULL = 96.0
# Gross pack size used to turn a raw-SOC slope into power. P550e: 38.2 kWh.
DEFAULT_PACK_KWH = 38.2
POWER_WINDOW_SECONDS = 15 * 60   # slope is taken over at most this long
POWER_MIN_SPAN_SECONDS = 120     # and needs at least this much time between samples

# 12V above this means the DC-DC converter is running: car on or charging.
DCDC_ACTIVE_VOLTS = 13.2

MODE_CHARGING = "charging"
MODE_DRIVING = "driving"
MODE_ON = "on"             # DC-DC running, stationary, battery not charging (e.g. preconditioning)
# Battery power (kW, from SOC slope) above which we call it charging.
CHARGING_POWER_KW = 0.4
MODE_IDLE = "idle"
MODE_ASLEEP = "asleep"
MODE_OUT_OF_RANGE = "out_of_range"
MODE_LOW_12V = "low_12v"
MODE_UNKNOWN = "unknown"


def determine_mode(data: dict[str, Any], low_12v_threshold: float) -> str:
    """Classify what the car is doing from one poll's decoded data.

    ``bat_12v_adapter`` (ATRV, no bus traffic) is available on every poll;
    ``bat_12v_voltage`` (OBD PID 42) only when the car answered.
    ``hv_power_est`` (kW, + charging / - discharging) comes from the SOC slope
    and is the charging signal, since the P550e exposes no charging DID.
    """
    if not data:
        return MODE_ASLEEP
    v12 = data.get("bat_12v_voltage")
    if not isinstance(v12, (int, float)):
        v12 = data.get("bat_12v_adapter")
    has_can = any(k not in ("bat_12v_adapter", "poll_mode", "soc_displayed", "hv_power_est") for k in data)
    speed = data.get("speed") or 0
    power = data.get("hv_power_est")
    dcdc_on = isinstance(v12, (int, float)) and v12 >= DCDC_ACTIVE_VOLTS
    if data.get("charging_status") == "charging":
        return MODE_CHARGING
    if speed > 0:
        return MODE_DRIVING
    if dcdc_on and isinstance(power, (int, float)) and power >= CHARGING_POWER_KW:
        return MODE_CHARGING
    if dcdc_on:
        return MODE_ON
    if isinstance(v12, (int, float)) and v12 < low_12v_threshold:
        return MODE_LOW_12V
    return MODE_IDLE if has_can else MODE_ASLEEP


def estimate_power_kw(samples, pack_kwh: float) -> float | None:
    """Battery power from the raw-SOC slope: positive = charging, negative = discharging.

    ``samples`` is an iterable of (unix_seconds, raw_soc_percent), oldest first.
    """
    pts = list(samples)
    if len(pts) < 2:
        return None
    (t0, s0), (t1, s1) = pts[0], pts[-1]
    span = t1 - t0
    if span < POWER_MIN_SPAN_SECONDS:
        return None
    pct_per_hour = (s1 - s0) / (span / 3600.0)
    return round(pct_per_hour / 100.0 * pack_kwh, 2)


def displayed_soc(raw: float | None, raw_empty: float, raw_full: float) -> float | None:
    """Map raw pack SOC onto the dashboard's 0–100 % usable window."""
    if raw is None or raw_full <= raw_empty:
        return None
    pct = (raw - raw_empty) / (raw_full - raw_empty) * 100.0
    return round(max(0.0, min(100.0, pct)), 1)


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
        self.sweep_running = False     # a DID sweep owns the BLE link; skip polls
        self._soc_samples: deque[tuple[float, float]] = deque()
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
        if self.sweep_running:
            return self._cache_data if self._cache_values else (self.data or {})
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
        wake = self._force_wake_next or self.poll_mode in (MODE_CHARGING, MODE_DRIVING, MODE_ON)
        self._force_wake_next = False

        try:
            new_data = await asyncio.wait_for(
                self.client.async_get_data(
                    self.options,
                    wake_ecus=wake,
                    passive_voltage_gate=None if wake else DCDC_ACTIVE_VOLTS,
                ),
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

        raw_soc = new_data.get("state_of_charge")
        if isinstance(raw_soc, (int, float)):
            now = datetime.now(timezone.utc).timestamp()
            self._soc_samples.append((now, float(raw_soc)))
            while self._soc_samples and now - self._soc_samples[0][0] > POWER_WINDOW_SECONDS:
                self._soc_samples.popleft()
            power = estimate_power_kw(self._soc_samples, self._pack_kwh)
            if power is not None:
                new_data["hv_power_est"] = power
            shown = displayed_soc(raw_soc, self._soc_raw_empty, self._soc_raw_full)
            if shown is not None:
                new_data["soc_displayed"] = shown
        else:
            # No SOC this poll (asleep / passive stop): a later slope across
            # the gap would be meaningless, so start over.
            self._soc_samples.clear()

        self.poll_mode = determine_mode(new_data, self._low_12v_threshold)
        if "charging_status" not in new_data and new_data:
            new_data["charging_status"] = (
                "charging" if self.poll_mode == MODE_CHARGING else "not_charging"
            )
        if self.poll_mode in (MODE_CHARGING, MODE_DRIVING, MODE_ON):
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
        else:  # asleep / parked with DC-DC off
            self._set_interval(self._xs_poll_interval, "Car parked and asleep")

        if self._cache_values:
            self._cache_data.update(new_data)
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
        self._soc_raw_empty = float(options.get("soc_raw_empty", DEFAULT_SOC_RAW_EMPTY))
        self._soc_raw_full = float(options.get("soc_raw_full", DEFAULT_SOC_RAW_FULL))
        self._pack_kwh = float(options.get("pack_kwh", DEFAULT_PACK_KWH))
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

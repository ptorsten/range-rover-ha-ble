"""Button platform for Range Rover BLE."""

import logging

from homeassistant.components.button import ButtonEntity
from homeassistant.components.persistent_notification import async_create
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant

from .const import DOMAIN, NAME
from .entity import RangeRoverBleEntity
from .obd_client import SLEEP_PROBE_COMMANDS
from .sweep import run_sweep

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(
    hass: HomeAssistant, entry: ConfigEntry, async_add_entities
) -> None:
    """Set up button platform."""
    coordinator = hass.data[DOMAIN][entry.entry_id]
    async_add_entities([
        RangeRoverBleRefreshButton(coordinator, entry),
        RangeRoverBleDiscoveryButton(coordinator, entry),
        RangeRoverBleAdapterProbeButton(coordinator, entry),
        RangeRoverBleSweepButton(coordinator, entry),
    ])


class RangeRoverBleRefreshButton(RangeRoverBleEntity, ButtonEntity):
    """Button that triggers an immediate coordinator refresh."""

    _attr_entity_category = EntityCategory.DIAGNOSTIC
    _attr_icon = "mdi:refresh"
    _attr_name = f"{NAME} Refresh"

    def __init__(self, coordinator, config_entry) -> None:
        """Initialize the button."""
        super().__init__(coordinator, config_entry)

    async def async_press(self) -> None:
        """Trigger an immediate update."""
        await self.coordinator.async_request_refresh()


class RangeRoverBleDiscoveryButton(RangeRoverBleEntity, ButtonEntity):
    """Button that runs PID discovery and posts results as a notification."""

    _attr_entity_category = EntityCategory.DIAGNOSTIC
    _attr_icon = "mdi:magnify-scan"
    _attr_name = f"{NAME} Run Discovery"

    def __init__(self, coordinator, config_entry) -> None:
        """Initialize the button."""
        super().__init__(coordinator, config_entry)

    async def async_press(self) -> None:
        """Run PID discovery and post results as a persistent notification."""
        _LOGGER.info("Starting PID discovery scan...")

        try:
            results = await self.coordinator.client.async_run_discovery()
        except Exception:
            _LOGGER.exception("Discovery scan failed")
            async_create(
                self.hass,
                "Discovery scan failed. Check the logs for details.",
                title="Range Rover BLE Discovery",
                notification_id="range_rover_ble_discovery",
            )
            return

        lines = ["## Range Rover BLE — PID Discovery Results\n"]

        responded = results.get("responded", [])
        no_data = results.get("no_data", [])
        all_errors = results.get("errors", [])
        nrc = [e for e in all_errors if e.get("nrc")]
        errors = [e for e in all_errors if not e.get("nrc")]

        if responded:
            lines.append(f"### {len(responded)} PID(s) returned data\n")
            for item in responded:
                label = item.get("label", "?")
                raw = item.get("raw", "")
                cmd = item.get("command", "")
                hdr = item.get("header", "")
                lines.append(f"- **{label}** [{hdr}/{cmd}]: `{raw}`")
        else:
            lines.append("### No PIDs returned data\n")

        if nrc:
            lines.append(f"\n### {len(nrc)} Negative Response Code(s)\n")
            for item in nrc:
                lines.append(f"- **{item.get('label', '?')}**: {item.get('nrc', '')}")

        if no_data:
            lines.append(f"\n### {len(no_data)} PID(s) returned NO DATA\n")
            for label in no_data:
                lines.append(f"- {label}")

        if errors:
            lines.append(f"\n### {len(errors)} Error(s)\n")
            for item in errors:
                label = item.get("label", "?")
                raw = item.get("raw", "")
                lines.append(f"- **{label}**: `{raw}`")

        body = "\n".join(lines)
        _LOGGER.info("Discovery complete: %d responded, %d no_data, %d nrc, %d errors",
                      len(responded), len(no_data), len(nrc), len(errors))

        async_create(
            self.hass,
            body,
            title="Range Rover BLE Discovery",
            notification_id="range_rover_ble_discovery",
        )


class RangeRoverBleAdapterProbeButton(RangeRoverBleEntity, ButtonEntity):
    """Button that asks the OBD adapter how it sleeps and posts the replies."""

    _attr_entity_category = EntityCategory.DIAGNOSTIC
    _attr_icon = "mdi:power-sleep"
    _attr_name = f"{NAME} Probe Adapter Sleep"

    def __init__(self, coordinator, config_entry) -> None:
        """Initialize the button."""
        super().__init__(coordinator, config_entry)

    async def async_press(self) -> None:
        """Send the sleep-probe command set and post a notification."""
        cmds = [c for c, _ in SLEEP_PROBE_COMMANDS]
        why = dict(SLEEP_PROBE_COMMANDS)
        try:
            replies = await self.coordinator.client.async_send_raw(
                cmds, self.coordinator.options
            )
        except Exception as err:  # noqa: BLE001
            _LOGGER.exception("Adapter probe failed")
            async_create(
                self.hass,
                f"Could not talk to the adapter: {err}",
                title="Range Rover BLE Adapter Probe",
                notification_id="range_rover_ble_adapter_probe",
            )
            return

        lines = ["## OBD adapter sleep probe\n"]
        is_stn = any(c == "STI" and not r.strip().startswith("?") and "<" not in r for c, r in replies)
        for cmd, reply in replies:
            lines.append(f"**`{cmd}`** — {why.get(cmd, '')}")
            lines.append("```\n" + reply + "\n```")
        if is_stn:
            lines.append(
                "\nThis is an **STN-based** adapter. Sleep is configurable: "
                "`STSLU off, off` disables link-inactivity sleep, `STSLVL off, off` "
                "disables voltage-based sleep; re-run this probe to confirm, then "
                "unplug/replug to check the settings persisted. Use the "
                "`range_rover_ble.send_raw_commands` service to send them."
            )
        else:
            lines.append(
                "\nNo STN firmware detected: this looks like a plain **ELM327 clone**. "
                "Sleep is governed by its own firmware; on genuine-firmware units "
                "programmable parameter 0E controls low power "
                "(`ATPP 0E SV 1A`, `ATPP 0E ON`, `ATZ` disables it). Clones often "
                "ignore it; check the vendor app for a sleep setting."
            )
        async_create(
            self.hass,
            "\n".join(lines),
            title="Range Rover BLE Adapter Probe",
            notification_id="range_rover_ble_adapter_probe",
        )


class RangeRoverBleSweepButton(RangeRoverBleEntity, ButtonEntity):
    """Button that sweeps DID ranges in the background and reports the diff."""

    _attr_entity_category = EntityCategory.DIAGNOSTIC
    _attr_icon = "mdi:radar"
    _attr_name = f"{NAME} Sweep DIDs"

    def __init__(self, coordinator, config_entry) -> None:
        """Initialize the button."""
        super().__init__(coordinator, config_entry)

    async def async_press(self) -> None:
        """Start the sweep without blocking the button press."""
        self.hass.async_create_background_task(
            run_sweep(self.hass, self.coordinator, self.config_entry.entry_id),
            "range_rover_ble DID sweep",
        )

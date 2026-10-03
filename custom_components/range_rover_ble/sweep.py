"""Run a DID sweep from Home Assistant, store it, and report the diff."""

from __future__ import annotations

from datetime import datetime, timezone
import logging

from homeassistant.components.persistent_notification import async_create
from homeassistant.core import HomeAssistant
from homeassistant.helpers.storage import Store

from .const import DOMAIN
from .obd_client import (
    DEFAULT_SWEEP_ECUS,
    DEFAULT_SWEEP_RANGES,
    compare_sweeps,
    guess_values,
    parse_did_ranges,
)

_LOGGER = logging.getLogger(__name__)
STORAGE_VERSION = 1
MAX_RUNS = 6
NOTIFICATION_ID = "range_rover_ble_sweep"


def _store(hass: HomeAssistant, entry_id: str) -> Store:
    return Store(hass, STORAGE_VERSION, f"{DOMAIN}.sweep.{entry_id}")


async def run_sweep(
    hass: HomeAssistant,
    coordinator,
    entry_id: str,
    ecus: list[str] | None = None,
    ranges=None,
    label: str | None = None,
) -> dict:
    """Sweep, persist, diff against the most recent run with a different label, notify."""
    ecus = [e.strip().upper() for e in (ecus or DEFAULT_SWEEP_ECUS)]
    ranges = ranges or DEFAULT_SWEEP_RANGES
    dids = parse_did_ranges(ranges)
    label = (label or coordinator.poll_mode or "unknown").strip()

    if coordinator.sweep_running:
        async_create(hass, "A DID sweep is already running.", title="Range Rover BLE DID sweep",
                     notification_id=NOTIFICATION_ID)
        return {}

    coordinator.sweep_running = True
    async_create(
        hass,
        f"Sweeping {len(dids)} DIDs on {', '.join(ecus)} (state: **{label}**). "
        f"This takes several minutes; polling is paused meanwhile.",
        title="Range Rover BLE DID sweep",
        notification_id=NOTIFICATION_ID,
    )

    def _progress(ecu, done, total, hits):
        _LOGGER.info("Sweep %s: %d/%d DIDs, %d answered", ecu, done, total, hits)

    try:
        results = await coordinator.client.async_sweep(ecus, dids, coordinator.options, _progress)
    except Exception as err:  # noqa: BLE001
        _LOGGER.exception("DID sweep failed")
        async_create(hass, f"DID sweep failed: {err}", title="Range Rover BLE DID sweep",
                     notification_id=NOTIFICATION_ID)
        return {}
    finally:
        coordinator.sweep_running = False

    store = _store(hass, entry_id)
    data = await store.async_load() or {"runs": []}
    run = {
        "label": label,
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "ecus": ecus,
        "ranges": ranges if isinstance(ranges, list) else [ranges],
        "results": results,
    }
    previous = next((r for r in reversed(data["runs"]) if r.get("label") != label), None)
    data["runs"] = (data["runs"] + [run])[-MAX_RUNS:]
    await store.async_save(data)

    total_hits = sum(len(v) for v in results.values())
    lines = [f"## DID sweep — state **{label}**", f"{total_hits} DID(s) answered across {', '.join(ecus)}.\n"]
    for ecu, hits in results.items():
        lines.append(f"**{ecu}**: {len(hits)} answered")
    if previous is None:
        lines.append(
            "\nNo earlier run in a different state to compare with yet. Run the sweep again "
            "when the car is in another state (e.g. unplugged after this charging run); the "
            "DIDs whose bytes change are the charging-status candidates."
        )
    else:
        diff = compare_sweeps(previous["results"], results)
        lines.append(f"\n### Compared with **{previous['label']}** run at {previous['timestamp']}")
        for ecu, d in diff.items():
            lines.append(
                f"\n**{ecu}** — {d['unchanged']} unchanged, {len(d['changed'])} changed, "
                f"{len(d['only_a'])} only before, {len(d['only_b'])} only now"
            )
            for ch in d["changed"]:
                lines.append(f"- `22{ch['did']}`  {ch['a']} → {ch['b']}  ({guess_values(ch['a'])} → {guess_values(ch['b'])})")
            for did in d["only_a"]:
                lines.append(f"- `22{did}` only in {previous['label']}: {previous['results'][ecu][did]['hex']}")
            for did in d["only_b"]:
                lines.append(f"- `22{did}` only now: {results[ecu][did]['hex']}")
    lines.append("\nFull results are stored in `.storage/range_rover_ble.sweep.*`.")
    body = "\n".join(lines)
    async_create(hass, body, title="Range Rover BLE DID sweep", notification_id=NOTIFICATION_ID)
    return run

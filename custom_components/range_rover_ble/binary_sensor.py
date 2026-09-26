"""Binary sensor platform for Range Rover BLE."""

from homeassistant.components.binary_sensor import (
    BinarySensorDeviceClass,
    BinarySensorEntity,
    BinarySensorEntityDescription,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant

from .const import DOMAIN, NAME
from .entity import RangeRoverBleEntity

BINARY_SENSOR_TYPES: dict[str, BinarySensorEntityDescription] = {
    "is_charging": BinarySensorEntityDescription(
        key="is_charging",
        device_class=BinarySensorDeviceClass.BATTERY_CHARGING,
        name="Charging",
    ),
}


async def async_setup_entry(
    hass: HomeAssistant, entry: ConfigEntry, async_add_devices
) -> bool:
    """Set up binary_sensor platform."""
    coordinator = hass.data[DOMAIN][entry.entry_id]
    entities = [
        RangeRoverBleBinarySensor(coordinator, entry, sensor_desc)
        for sensor_desc in BINARY_SENSOR_TYPES
    ]
    async_add_devices(entities)


class RangeRoverBleBinarySensor(RangeRoverBleEntity, BinarySensorEntity):
    """Range Rover BLE binary sensor."""

    def __init__(
        self,
        coordinator,
        config_entry,
        sensor: str,
    ) -> None:
        """Initialize the binary sensor."""
        super().__init__(coordinator, config_entry)
        self._sensor = sensor
        self._attr_name = f"{NAME} {BINARY_SENSOR_TYPES[sensor].name}"
        self._attr_device_class = BINARY_SENSOR_TYPES[sensor].device_class

    @property
    def is_on(self):
        """Return true if charging."""
        charging_status = self.coordinator.data.get("charging_status")
        if self._sensor == "is_charging":
            return charging_status == "charging"
        return self.coordinator.data.get(self._sensor)

    @property
    def icon(self):
        """Return the icon of the sensor."""
        return BINARY_SENSOR_TYPES[self._sensor].icon

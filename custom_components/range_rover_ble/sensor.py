"""Sensor platform for Range Rover BLE."""

from homeassistant.components.sensor import (
    SensorDeviceClass,
    SensorEntity,
    SensorEntityDescription,
    SensorStateClass,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.const import EntityCategory
from homeassistant.core import HomeAssistant

from .const import DOMAIN, NAME
from .entity import RangeRoverBleEntity

SENSOR_TYPES: dict[str, SensorEntityDescription] = {
    "state_of_charge": SensorEntityDescription(
        key="state_of_charge",
        icon="mdi:battery-charging",
        name="HV battery SOC",
        native_unit_of_measurement="%",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.BATTERY,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "soc_min": SensorEntityDescription(
        key="soc_min",
        icon="mdi:battery-low",
        name="HV battery SOC (min cell)",
        native_unit_of_measurement="%",
        suggested_display_precision=1,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "soc_std": SensorEntityDescription(
        key="soc_std",
        icon="mdi:battery",
        name="Hybrid battery remaining (OBD std)",
        native_unit_of_measurement="%",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.BATTERY,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "soh_min": SensorEntityDescription(
        key="soh_min",
        icon="mdi:battery-heart-variant",
        name="HV battery state of health (min)",
        native_unit_of_measurement="%",
        suggested_display_precision=1,
        state_class=SensorStateClass.MEASUREMENT,
        entity_registry_enabled_default=False,
    ),
    "soh_max": SensorEntityDescription(
        key="soh_max",
        icon="mdi:battery-heart-variant",
        name="HV battery state of health (max)",
        native_unit_of_measurement="%",
        suggested_display_precision=1,
        state_class=SensorStateClass.MEASUREMENT,
        entity_registry_enabled_default=False,
    ),
    "battery_plate_temp_1": SensorEntityDescription(
        key="battery_plate_temp_1",
        name="HV battery plate temperature 1",
        native_unit_of_measurement="°C",
        device_class=SensorDeviceClass.TEMPERATURE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "battery_plate_temp_2": SensorEntityDescription(
        key="battery_plate_temp_2",
        name="HV battery plate temperature 2",
        native_unit_of_measurement="°C",
        device_class=SensorDeviceClass.TEMPERATURE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "charge_indicator": SensorEntityDescription(
        key="charge_indicator",
        icon="mdi:ev-station",
        name="Charge indicator (DD06, experimental)",
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "poll_mode": SensorEntityDescription(
        key="poll_mode",
        icon="mdi:car-clock",
        name="Poll mode",
        entity_category=EntityCategory.DIAGNOSTIC,
    ),
    "soc_displayed": SensorEntityDescription(
        key="soc_displayed",
        icon="mdi:battery-charging-70",
        name="Battery charge (displayed, estimated)",
        native_unit_of_measurement="%",
        suggested_display_precision=0,
        device_class=SensorDeviceClass.BATTERY,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "bat_12v_adapter": SensorEntityDescription(
        key="bat_12v_adapter",
        name="12V battery voltage (adapter)",
        native_unit_of_measurement="V",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.VOLTAGE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "hv_power_est": SensorEntityDescription(
        key="hv_power_est",
        icon="mdi:flash",
        name="HV battery power (estimated)",
        native_unit_of_measurement="kW",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.POWER,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "hv_battery_voltage": SensorEntityDescription(
        key="hv_battery_voltage",
        name="HV battery voltage",
        native_unit_of_measurement="V",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.VOLTAGE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "hv_battery_current": SensorEntityDescription(
        key="hv_battery_current",
        name="HV battery current",
        native_unit_of_measurement="A",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.CURRENT,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "hv_battery_temp": SensorEntityDescription(
        key="hv_battery_temp",
        icon="mdi:thermometer",
        name="HV battery temperature",
        native_unit_of_measurement="°C",
        suggested_display_precision=0,
        device_class=SensorDeviceClass.TEMPERATURE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "soh_avg": SensorEntityDescription(
        key="soh_avg",
        icon="mdi:battery-heart",
        name="HV battery state of health",
        native_unit_of_measurement="%",
        suggested_display_precision=1,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "cell_voltage_max": SensorEntityDescription(
        key="cell_voltage_max",
        name="Cell voltage (max)",
        native_unit_of_measurement="V",
        suggested_display_precision=3,
        device_class=SensorDeviceClass.VOLTAGE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "cell_voltage_min": SensorEntityDescription(
        key="cell_voltage_min",
        name="Cell voltage (min)",
        native_unit_of_measurement="V",
        suggested_display_precision=3,
        device_class=SensorDeviceClass.VOLTAGE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "bat_12v_voltage": SensorEntityDescription(
        key="bat_12v_voltage",
        icon="mdi:car-battery",
        name="12V battery voltage",
        native_unit_of_measurement="V",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.VOLTAGE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "ambient_temp": SensorEntityDescription(
        key="ambient_temp",
        icon="mdi:thermometer",
        name="Ambient temperature",
        native_unit_of_measurement="°C",
        suggested_display_precision=0,
        device_class=SensorDeviceClass.TEMPERATURE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "speed": SensorEntityDescription(
        key="speed",
        icon="mdi:speedometer",
        name="Vehicle speed",
        native_unit_of_measurement="km/h",
        suggested_display_precision=0,
        device_class=SensorDeviceClass.SPEED,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "bccm_soc": SensorEntityDescription(
        key="bccm_soc",
        icon="mdi:battery-charging",
        name="HV battery SOC (BCCM)",
        native_unit_of_measurement="%",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.BATTERY,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "bccm_voltage": SensorEntityDescription(
        key="bccm_voltage",
        name="HV battery voltage (BCCM)",
        native_unit_of_measurement="V",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.VOLTAGE,
        state_class=SensorStateClass.MEASUREMENT,
    ),
    "bccm_current": SensorEntityDescription(
        key="bccm_current",
        name="HV battery current (BCCM)",
        native_unit_of_measurement="A",
        suggested_display_precision=1,
        device_class=SensorDeviceClass.CURRENT,
        state_class=SensorStateClass.MEASUREMENT,
    ),
}


async def async_setup_entry(
    hass: HomeAssistant, entry: ConfigEntry, async_add_entities
):
    """Set up sensor platform."""
    coordinator = hass.data[DOMAIN][entry.entry_id]
    entities = [
        RangeRoverBleSensor(coordinator, entry, desc)
        for desc in SENSOR_TYPES.values()
    ]
    async_add_entities(entities)


class RangeRoverBleSensor(RangeRoverBleEntity, SensorEntity):
    """Range Rover BLE sensor entity."""

    def __init__(
        self,
        coordinator,
        config_entry,
        description: SensorEntityDescription,
    ) -> None:
        """Initialize the sensor."""
        super().__init__(coordinator, config_entry)
        self._description = description
        self._sensor = description.key
        self._attr_name = f"{NAME} {description.name}"
        self._attr_device_class = description.device_class
        self._attr_native_unit_of_measurement = description.native_unit_of_measurement
        self._attr_state_class = description.state_class

    @property
    def native_value(self):
        """Return the state of the sensor."""
        return self.coordinator.data.get(self._sensor)

    @property
    def icon(self):
        """Return the icon of the sensor."""
        return self._description.icon

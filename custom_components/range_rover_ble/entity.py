"""RangeRoverBleEntity class."""

from homeassistant.const import CONF_ADDRESS
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .const import DOMAIN, NAME, VERSION


class RangeRoverBleEntity(CoordinatorEntity):
    """Base entity for Range Rover BLE."""

    def __init__(self, coordinator, config_entry) -> None:
        """Initialise."""
        super().__init__(coordinator)
        self.config_entry = config_entry

    @property
    def unique_id(self):
        """Return a unique ID to use for this entity."""
        return f"{self.config_entry.data[CONF_ADDRESS]}-{self.name}"

    @property
    def device_info(self):
        """Return device information."""
        return {
            "identifiers": {(DOMAIN, self.config_entry.data[CONF_ADDRESS])},
            "name": NAME,
            "model": "P550e PHEV",
            "manufacturer": "Land Rover",
        }

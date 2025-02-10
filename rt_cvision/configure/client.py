import django
django.setup()

from tenants.models import (
    PlantEntity,
    SensorBox,
)

class ServiceConfig:
    """Represents a single service configuration allowing dot-notation access."""
    def __init__(self, params):
        self.__dict__.update(params)  # Convert dict keys to attributes

    def __getattr__(self, name):
        """Return None instead of raising AttributeError if parameter is missing."""
        return None

    def __repr__(self):
        return f"<ServiceConfig {self.__dict__}>"
    
class ConfigManager:
    """Django Service Configuration Manager that allows dot notation access."""
    _instance = None  # Singleton instance

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConfigManager, cls).__new__(cls)
            cls._instance.reload_config()
        return cls._instance

    def reload_config(self):
        """Reload configuration from the database."""
        from configure.models import Service, ServiceParams  # Avoid early import issues

        self.services = Service.objects.all()
        self.__dict__.update({
            service.service_name: ServiceConfig({
                param.name: param.value for param in ServiceParams.objects.filter(service=service)
            })
            for service in self.services
        })

    def __repr__(self):
        return f"<ConfigManager Services={list(self.__dict__.keys())}>"

entity = PlantEntity.objects.all()
sensorbox = None
if entity:
    entity = entity.first()
    sensorbox = SensorBox.objects.get(plant_entity=entity)

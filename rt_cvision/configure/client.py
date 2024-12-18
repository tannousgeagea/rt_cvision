import django
django.setup()
from configure.models import Service, ServiceParams
from impurity.models import (
    Impurity
)

from data_reader.models import (
    Image
)

from tenants.models import (
    Tenant,
    EntityType,
    PlantEntity,
    SensorBox,
)

class ConfigManager:
    def __init__(self):            
        self.params = {}
        self.services = Service.objects.all()
        for service in self.services:
            self.params[service.service_name] = {
                param.name: param.value
                for param in ServiceParams.objects.filter(service=service)
                } 
    

config_manager = ConfigManager()
entity = PlantEntity.objects.all()
sensorbox = None
if entity:
    entity = entity.first()
    sensorbox = SensorBox.objects.get(plant_entity=entity)
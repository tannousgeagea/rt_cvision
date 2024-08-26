import django
django.setup()
from configure.models import Service, ServiceParams

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

print(config_manager.params.get('Segments'))
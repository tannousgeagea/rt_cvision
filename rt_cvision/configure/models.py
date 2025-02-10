from django.db import models

# Create your models here.
class Service(models.Model):
    service_id = models.CharField(max_length=255)
    service_name = models.CharField(max_length=255)
    description = models.CharField(max_length=255)
    created_at = models.DateTimeField(auto_now_add=True)
    meta_info = models.JSONField(null=True, blank=True)
    
    class Meta:
        db_table = 'service'
        verbose_name = "Service"
        verbose_name_plural = "Services"
        
    def __str__(self) -> str:
        return f"{self.service_name}"
    
    
class ServiceParams(models.Model):
    VALUE_TYPES = [
        ('str', 'String'),
        ('int', 'Integer'),
        ('float', 'Float'),
        ('bool', 'Boolean'),
        ('json', 'JSON'),
    ]
    
    service = models.ForeignKey(Service, on_delete=models.CASCADE)
    name = models.CharField(max_length=255)
    value_type = models.CharField(max_length=255, choices=VALUE_TYPES)
    input_type = models.CharField(max_length=255, null=True, blank=True)
    value = models.JSONField(null=True, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    description = models.CharField(max_length=255)
    meta_info = models.JSONField(null=True, blank=True)
    
    class Meta:
        db_table = 'service_parameters'
        verbose_name_plural = 'Service Parameters'
    
    def __str__(self) -> str:
        return f'{self.service}, {self.name}, {self.value_type}'

    def save(self, *args, **kwargs):
        if self.value_type == 'str' and not isinstance(self.value, str):
            raise ValueError('Value must be a string')
        elif self.value_type == 'int' and not isinstance(self.value, int):
            raise ValueError('Value must be an integer')
        elif self.value_type == 'float' and not isinstance(self.value, float):
            raise ValueError('Value must be a float')
        elif self.value_type == 'bool' and not isinstance(self.value, bool):
            raise ValueError('Value must be a boolean')

        super().save(*args, **kwargs)

class AppConfig(models.Model):
    """Stores global application configuration status."""
    is_configured = models.BooleanField(default=False)  # False until configured
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    
    @classmethod
    def is_app_ready(cls):
        """Returns True if the app is fully configured."""
        return cls.objects.filter(is_configured=True).exists()

class DataSource(models.Model):
    """Represents a generic data source (ROS2 topic, MQTT topic, API endpoint, etc.)"""
    
    INTERFACE_CHOICES = [
        ('ros2', 'ROS2 Topic'),
        ('mqtt', 'MQTT Topic'),
        ('api', 'REST API Endpoint'),
        ('websocket', 'WebSocket Feed'),
        ('file', 'File Source'),
    ]

    name = models.CharField(max_length=255, unique=True) 
    interface = models.CharField(max_length=50, choices=INTERFACE_CHOICES) 
    is_available = models.BooleanField(default=False)
    last_detected = models.DateTimeField(auto_now=True)

    def __str__(self):
        return f"{self.name} ({self.get_interface_display()})"

    @classmethod
    def get_available_sources(cls, interface):
        """Return available data sources for a given interface type"""
        return cls.objects.filter(interface=interface, is_available=True)

class DataAcquisitionConfig(models.Model):
    """Stores the selected data source for the data acquisition app"""
    selected_source = models.ForeignKey(DataSource, on_delete=models.SET_NULL, null=True)

    def __str__(self):
        return f"Using: {self.selected_source.name if self.selected_source else 'None'}"
    
    @classmethod
    def get_selected_source(cls):
        """Returns the currently selected data source"""
        config = cls.objects.first()
        return config.selected_source if config else None
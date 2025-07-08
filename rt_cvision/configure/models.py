from django.db import models
import common_utils.health.runtime

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


# -------------------------------------------------------------------
# 1. Data Type
# -------------------------------------------------------------------
class ValueType(models.Model):
    name = models.CharField(max_length=50, unique=True)
    description = models.TextField(blank=True, null=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        verbose_name = "Value Type"
        verbose_name_plural = "Value Types"
        ordering = ['name']

    def __str__(self):
        return self.name


class InputType(models.Model):
    name = models.CharField(max_length=50, unique=True)
    description = models.TextField(blank=True, null=True)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)

    class Meta:
        verbose_name = "Input Type"
        verbose_name_plural = "Input Types"
        ordering = ['name']

    def __str__(self):
        return self.name

# -------------------------------------------------------------------
# config Group Models
# -------------------------------------------------------------------
class ServiceConfigGroup(models.Model):
    """
    Represents a group (or section) of configuration fields for a given service.
    For example: 'General Settings' or 'Performance Settings'.
    """
    service = models.ForeignKey(
        Service, 
        on_delete=models.RESTRICT, 
        related_name='config_groups'
    )
    name = models.CharField(max_length=255)
    order = models.PositiveIntegerField(default=0)
    meta_info = models.JSONField(null=True, blank=True)
    is_active = models.BooleanField(default=True, help_text="Indicates if the parameter is currently active.")

    class Meta:
        db_table = 'service_config_group'
        verbose_name = "Service Configuration Group"
        verbose_name_plural = "Service Configuration Groups"
        ordering = ['order']

    def __str__(self):
        return f"{self.service.service_name} - {self.name}"

# -------------------------------------------------------------------
# Field Definition & Field Instance Models
# -------------------------------------------------------------------
class ConfigFieldDefinition(models.Model):
    """
    Master definition of a configuration field. These definitions are shared across services.
    """
    field_id = models.CharField(
        max_length=100,
        help_text="A unique identifier for the field (e.g., 'auth-mode')."
    )
    label = models.CharField(max_length=255)
    value_type = models.ForeignKey(ValueType, on_delete=models.PROTECT)
    input_type = models.ForeignKey(InputType, on_delete=models.PROTECT, null=True, blank=True)
    default_value = models.JSONField(
        null=True,
        blank=True,
        help_text="Default value for the field."
    )
    validation = models.JSONField(
        null=True,
        blank=True,
        help_text="Validation rules (e.g., {'required': True, 'min': 1, 'max': 168})."
    )
    description = models.TextField(blank=True, null=True)
    meta_info = models.JSONField(null=True, blank=True)
    options = models.JSONField(
        null=True,
        blank=True,
        help_text="For select or multiselect input types, an array of available options. "
                  "Each option can be an object with keys like 'id' and 'label'."
    )
    is_active = models.BooleanField(default=True, help_text="Indicates if the parameter is currently active.")
    
    class Meta:
        db_table = 'config_field_definition'
        verbose_name = "Configuration Field Definition"
        verbose_name_plural = "Configuration Field Definitions"
        unique_together = (("field_id",),)

    def __str__(self):
        return f"{self.label} ({self.field_id})"

# -------------------------------------------------------------------
# Field Instance Models
# -------------------------------------------------------------------
class ServiceConfigFieldInstance(models.Model):
    """
    A service-specific instance of a configuration field.
    References a master definition and stores the actual value used by the service.
    """
    group = models.ForeignKey(
        ServiceConfigGroup, 
        on_delete=models.RESTRICT, 
        related_name='fields'
    )
    definition = models.ForeignKey(
        ConfigFieldDefinition, 
        on_delete=models.PROTECT,
        help_text="Reference to the shared field definition."
    )
    value = models.JSONField(
        null=True,
        blank=True,
        help_text="The value for this field instance. If not set, the default from the definition may be used."
    )
    meta_info = models.JSONField(null=True, blank=True)
    order = models.PositiveIntegerField(default=0)
    is_active = models.BooleanField(default=True, help_text="Indicates if the parameter is currently active.")
    options = models.JSONField(
        null=True,
        blank=True,
        help_text="Service-specific options for select/multiselect inputs. Overrides the default definition if set."
    )
    
    class Meta:
        db_table = 'service_config_field_instance'
        verbose_name = "Service Configuration Field Instance"
        verbose_name_plural = "Service Configuration Field Instances"
        ordering = ['order']
        unique_together = (("group", "definition"),)

    def __str__(self):
        return f"{self.group.name} - {self.definition.label}"

    def save(self, *args, **kwargs):
        # If no value is provided, optionally fallback to default_value from the definition.
        if self.value is None and self.definition.default_value is not None:
            self.value = self.definition.default_value
        
        # Validate the type according to the definition's value_type.
        vt = self.definition.value_type.name.lower()
        if self.value is not None:
            if vt in ['string', 'str'] and not isinstance(self.value, str):
                raise ValueError('Value must be a string')
            elif vt in ['integer', 'int'] and not isinstance(self.value, int):
                raise ValueError('Value must be an integer')
            elif vt == 'float' and not isinstance(self.value, float):
                raise ValueError('Value must be a float')
            elif vt in ['boolean', 'bool'] and not isinstance(self.value, bool):
                raise ValueError('Value must be a boolean')
        super().save(*args, **kwargs)

##############################################################################################
################################### App Config ###############################################
##############################################################################################
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
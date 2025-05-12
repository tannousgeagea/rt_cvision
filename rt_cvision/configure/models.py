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
    is_active = models.BooleanField(default=True, help_text="Indicates if the parameter is currently active.")
    
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
    
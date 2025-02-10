from django.db import models
from data_reader.models import (
    Image
)

class Impurity(models.Model):
    image = models.ForeignKey(Image, on_delete=models.RESTRICT, related_name='image')
    object_uid = models.CharField(max_length=255)
    timestamp = models.DateTimeField()
    confidence_score = models.FloatField()
    class_id = models.IntegerField()
    object_length = models.FloatField(null=True, blank=True)
    object_coordinates = models.JSONField(null=True, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    is_processed = models.BooleanField(default=False)
    class Meta:
        db_table = 'impurity'
        verbose_name_plural = 'Impurities'
        
    def __str__(self):
        return f"Object {self.object_uid} ({self.class_id}) ({self.object_uid})"
    
    def save(self, *args, **kwargs):
        super().save(*args, **kwargs)
        
class ImpurityTask(models.Model):
    name = models.CharField(max_length=100, unique=True)
    description = models.CharField(max_length=255, null=True, blank=True)
    is_enabled = models.BooleanField(default=True)
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        db_table = "impurity_task"
        verbose_name_plural = "Impurity Tasks"
    
    def __str__(self):
        return self.name
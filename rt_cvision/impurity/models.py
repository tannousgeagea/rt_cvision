from django.db import models
from data_reader.models import (
    Image
)

from .signals import queue_impurity_event

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
        is_new = self.pk is None
        super().save(*args, **kwargs) 
        if is_new:
            try:
                queue_impurity_event(self)
            except Exception as e:
                # Optional: logging instead of print
                print(f"Redis push failed: {e}")
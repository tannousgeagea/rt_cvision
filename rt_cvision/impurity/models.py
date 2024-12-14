from django.db import models
from data_reader.models import (
    Image
)

class Impurity(models.Model):
    image = models.ForeignKey(Image, on_delete=models.RESTRICT, related_name='image')
    object_uid = models.CharField(max_length=255)
    timestamp = models.DateTimeField()
    tracker_id = models.IntegerField(null=True, blank=True)
    confidence_score = models.FloatField()
    class_id = models.IntegerField()
    object_length = models.FloatField(null=True, blank=True)
    object_coordinates = models.JSONField(null=True, blank=True)
    
    class Meta:
        db_table = 'impurity'
        verbose_name_plural = 'Impurities'
        
    def __str__(self):
        return f"Object {self.object_uid} ({self.class_id}) ({self.object_uid})"
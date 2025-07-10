from django.db import models
from metadata.models import Tag
from data_reader.models import (
    Image
)
from .signals import queue_impurity_event
from django.contrib.auth import get_user_model
User = get_user_model()

class Impurity(models.Model):
    image = models.ForeignKey(Image, on_delete=models.RESTRICT, related_name='impurities')
    object_uid = models.CharField(max_length=255)
    timestamp = models.DateTimeField()
    confidence_score = models.FloatField()
    class_id = models.IntegerField()
    object_length = models.FloatField(null=True, blank=True)
    object_coordinates = models.JSONField(null=True, blank=True)
    created_at = models.DateTimeField(auto_now_add=True)
    is_processed = models.BooleanField(default=False)
    meta_info = models.JSONField(null=True, blank=True)
    tags = models.ManyToManyField(
        Tag,
        through='ImpurityTag',
        related_name='impurities',
        blank=True
    )

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
        
class ImpurityTag(models.Model):
    impurity = models.ForeignKey(Impurity, on_delete=models.CASCADE, related_name='impurity_tags')
    tag = models.ForeignKey(Tag, on_delete=models.CASCADE, related_name='tagged_impurities')
    tagged_by = models.ForeignKey(User, on_delete=models.SET_NULL, null=True, blank=True)
    tagged_at = models.DateTimeField(auto_now_add=True)
    source = models.CharField(
        max_length=50,
        choices=[('auto', 'Auto'), ('human', 'Human'), ('model', 'Model')],
        default='auto'
    )
    confidence = models.FloatField(null=True, blank=True)

    class Meta:
        unique_together = ('impurity', 'tag')
        db_table = 'impurity_tag'
        verbose_name_plural = 'Impurity Tags'
        indexes = [
            models.Index(fields=['tag']),
            models.Index(fields=['impurity']),
        ]

    def __str__(self):
        return f"{self.impurity.object_uid} - {self.tag.name}"

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
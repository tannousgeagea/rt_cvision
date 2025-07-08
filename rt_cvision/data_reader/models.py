from django.db import models
from metadata.models import Tag
from tenants.models import (
    SensorBox
)
from django.contrib.auth import get_user_model
User = get_user_model()

def get_image_path(instance, filename):
    return f"images/{filename}"

class Image(models.Model):
    image_id = models.CharField(max_length=255, unique=True)
    image_name = models.CharField(max_length=255)
    image_file = models.ImageField(upload_to=get_image_path)
    image_size = models.IntegerField(null=True, blank=True)  # Size in bytes
    image_format = models.CharField(max_length=50, null=True, blank=True)  # JPEG, PNG, etc.
    timestamp = models.DateTimeField()  # Time of capture
    created_at = models.DateTimeField(auto_now_add=True)
    is_processed = models.BooleanField(default=False)
    expires_at = models.DateTimeField(null=True, blank=True)  # Expiration time for cleanup
    source = models.CharField(max_length=255, null=True, blank=True)  # Optional source of the image
    meta_info = models.JSONField(null=True, blank=True)
    sensorbox = models.ForeignKey(SensorBox, on_delete=models.SET_NULL, null=True)  # New relationship
    width = models.PositiveIntegerField(null=True, blank=True, help_text="Original width in pixels")
    height = models.PositiveIntegerField(null=True, blank=True, help_text="Original height in pixels")
    tags = models.ManyToManyField(
        Tag,
        through='ImageTag',
        related_name='images',
        blank=True
    )
    class Meta:
        db_table = 'image'
        verbose_name = 'Image'
        verbose_name_plural = "Images"

    def __str__(self) -> str:
        return f"Image: {self.image_id} created at {self.created_at}"

class ImageTag(models.Model):
    image = models.ForeignKey(Image, on_delete=models.CASCADE, related_name='image_tags')
    tag = models.ForeignKey(Tag, on_delete=models.CASCADE, related_name='tagged_images')
    tagged_by = models.ForeignKey(User, on_delete=models.SET_NULL, null=True, blank=True)
    tagged_at = models.DateTimeField(auto_now_add=True)
    source = models.CharField(
        max_length=50,
        choices=[('auto', 'Auto'), ('human', 'Human'), ('model', 'Model')],
        default='auto'
    )
    confidence = models.FloatField(null=True, blank=True)
    
    class Meta:
        unique_together = ('image', 'tag')
        db_table = 'image_tag'
        verbose_name_plural = 'Image Tags'

    def __str__(self):
        return f"{self.image.image_name} - {self.tag.name}"
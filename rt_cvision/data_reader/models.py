from django.db import models

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

    class Meta:
        db_table = 'image'
        verbose_name = 'Image'
        verbose_name_plural = "Images"

    def __str__(self) -> str:
        return f"Image: {self.image_id} created at {self.created_at}"
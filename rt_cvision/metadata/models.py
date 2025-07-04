import string
from django.db import models


TAG_GROUP_MAP = {
    'severity': lambda name: name.startswith("predicted_severity:") or name in ['low', 'medium', 'high'],
    'material': lambda name: name in ["glass", "plastic", "metal", ""],
    'geometry': lambda name: name in ["rigid", "sharp", "elongated", "manmade", "round", "long_object", "long", "short"],
}

class Language(models.Model):
    """
    Model to define and manage supported languages.
    """
    code = models.CharField(max_length=10, unique=True)  # ISO 639-1 language codes, e.g., 'en', 'fr'
    name = models.CharField(max_length=50)  # e.g., 'English', 'French'
    created_at = models.DateTimeField(auto_now_add=True)

    class Meta:
        db_table = 'language'
        verbose_name_plural = 'Languages'

    def __str__(self):
        return f"{self.name} ({self.code})"
    
class TagGroup(models.Model):
    name = models.CharField(max_length=100, unique=True)
    description = models.TextField(blank=True, null=True)

    class Meta:
        db_table = 'tag_group'
        verbose_name = 'Tag Group'
        verbose_name_plural = 'Tag Groups'

    def __str__(self):
        return self.name

def infer_tag_group(name: str) -> TagGroup | None:
    for group_name, condition in TAG_GROUP_MAP.items():
        if condition(name):
            return TagGroup.objects.get_or_create(name=group_name)[0]
    return None    

class Tag(models.Model):
    name = models.CharField(max_length=100, unique=True)
    tag_type = models.CharField(max_length=50, blank=True, null=True)  # e.g. 'material', 'context', 'priority'
    created_at = models.DateTimeField(auto_now_add=True)
    description = models.TextField(blank=True)
    group = models.ForeignKey(
        TagGroup,
        on_delete=models.SET_NULL,
        null=True,
        blank=True,
        related_name='tags'
    )
    color = models.CharField(max_length=10, blank=True, null=True)  # e.g. "#00FF00"

    def save(self, *args, **kwargs):
        if self.group is None:
            self.group = infer_tag_group(self.name)
        super().save(*args, **kwargs)

    class Meta:
        db_table = 'tag'
        verbose_name_plural = 'Tags'

    def __str__(self):
        return self.name
    
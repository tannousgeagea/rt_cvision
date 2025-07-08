from django.contrib import admin
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import (
    Image,
    ImageTag
)

class ImageTagInline(TabularInline):
    model = ImageTag
    extra = 1
    autocomplete_fields = ['tag', 'tagged_by']

@admin.register(Image)
class ImageAdmin(ModelAdmin):
    list_display = ('image_id', 'image_name', 'timestamp', 'source', 'is_processed')
    list_filter = ('is_processed', 'source')
    search_fields = ('image_id', 'image_name')
    inlines = [ImageTagInline]

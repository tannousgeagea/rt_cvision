from django.contrib import admin
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import (
    Image
)

@admin.register(Image)
class ImageAdmin(ModelAdmin):
    list_display = ('image_id', 'image_name', 'image_file')
    search_fields = ('image_id', "image_name")

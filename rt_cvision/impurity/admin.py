from django.contrib import admin
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import (
    Impurity
)

@admin.register(Impurity)
class ImpurityAdmin(ModelAdmin):
    list_display = ('image', 'object_uid', 'confidence_score', 'class_id', 'is_processed')
    list_filter = ('is_processed', )
    search_fields = ("object_uid", 'image__image_id')
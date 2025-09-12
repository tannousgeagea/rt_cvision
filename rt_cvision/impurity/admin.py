from django.contrib import admin
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import (
    Impurity,
    ImpurityTask,
    ImpurityTag,
)

class ImpurityTagInline(TabularInline):
    model = ImpurityTag
    extra = 1
    autocomplete_fields = ['tag', 'tagged_by']


@admin.register(Impurity)
class ImpurityAdmin(ModelAdmin):
    list_display = ('image', 'object_uid', 'confidence_score', 'class_id', 'timestamp', 'is_processed')
    list_filter = ('is_processed', 'class_id', 'tags')
    search_fields = ("object_uid", 'image__image_id', "image__image_name")
    inlines = [ImpurityTagInline]
    
@admin.register(ImpurityTask)
class ImpurityTaskAdmin(ModelAdmin):
    list_display = ("name", "is_enabled", "created_at")
from django.contrib import admin
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import (
    Language,
    TagGroup, 
    Tag,
)

@admin.register(Language)
class LanguageAdmin(ModelAdmin):
    list_display = ('code', 'name', 'created_at')
    search_fields = ('code', 'name')
    ordering = ('-created_at',)

@admin.register(TagGroup)
class TagGroupAdmin(ModelAdmin):
    list_display = ('name',)
    search_fields = ('name',)


@admin.register(Tag)
class TagAdmin(ModelAdmin):
    list_display = ('name', 'group', 'color')
    list_filter = ('group',)
    search_fields = ('name',)
    autocomplete_fields = ('group',)
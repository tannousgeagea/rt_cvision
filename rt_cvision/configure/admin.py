from django.contrib import admin
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import Service, ServiceParams

class ServiceParamsInline(TabularInline):
    model = ServiceParams

@admin.register(Service)
class ServiceAdmin(ModelAdmin):
    list_display = ('service_id', 'service_name', 'description', 'created_at')
    search_fields = ('service_id', 'service_name', 'description')
    list_filter = ('created_at',)
    readonly_fields = ('created_at',)
    fieldsets = (
        (None, {
            'fields': ('service_id', 'service_name', 'description', 'meta_info')
        }),
        ('Timestamps', {
            'fields': ('created_at',),
        }),
    )
    
    inlines = [ServiceParamsInline]

@admin.register(ServiceParams)
class ServiceParamsAdmin(ModelAdmin):
    list_display = ('service', 'name', 'value_type', 'input_type', 'created_at')
    search_fields = ('service__service_name', 'name', 'description')
    list_filter = ('value_type', 'created_at', "service__service_name")
    readonly_fields = ('created_at',)
    fieldsets = (
        (None, {
            'fields': ('service', 'name', 'value_type', 'input_type', 'value', 'description', 'is_active', 'meta_info')
        }),
        ('Timestamps', {
            'fields': ('created_at',),
        }),
    )

    def save_model(self, request, obj, form, change):
        # Ensure that the value type matches the provided value before saving
        if obj.value_type == 'str' and not isinstance(obj.value, str):
            raise ValueError('Value must be a string')
        elif obj.value_type == 'int' and not isinstance(obj.value, int):
            raise ValueError('Value must be an integer')
        elif obj.value_type == 'float' and not isinstance(obj.value, float):
            raise ValueError('Value must be a float')
        elif obj.value_type == 'bool' and not isinstance(obj.value, bool):
            raise ValueError('Value must be a boolean')

        super().save_model(request, obj, form, change)

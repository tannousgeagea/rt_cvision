from django.contrib import admin
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import (
    Service, 
    ServiceParams, 
    AppConfig,
    DataSource,
    DataAcquisitionConfig,
)

from .models import (
    ServiceConfigGroup,
    ServiceConfigFieldInstance,
    ConfigFieldDefinition,
    ValueType,
    InputType,
)

class ServiceParamsInline(TabularInline):
    model = ServiceParams

class ServiceConfigFieldInstanceInline(TabularInline):
    model = ServiceConfigFieldInstance
    extra = 1
    fields = ('definition', 'value', 'order', 'meta_info')
    ordering = ('order',)
    
class ServiceConfigGroupInline(TabularInline):
    model = ServiceConfigGroup
    extra = 1
    fields = ('name', 'order', 'meta_info')
    ordering = ('order',)

@admin.register(ValueType)
class ValueTypeAdmin(ModelAdmin):
    list_display = ("name", "description", "created_at", "updated_at")

@admin.register(InputType)
class InputTypeAdmin(ModelAdmin):
    list_display = ("name", "description", "created_at", "updated_at")

@admin.register(ServiceConfigGroup)
class ServiceConfigGroupAdmin(admin.ModelAdmin):
    inlines = [ServiceConfigFieldInstanceInline]
    list_display = ('name', 'service', 'order')
    list_filter = ('service',)
    ordering = ('order',)
    
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
    
    inlines = [ServiceConfigGroupInline, ServiceParamsInline]

@admin.register(ServiceParams)
class ServiceParamsAdmin(ModelAdmin):
    list_display = ('service', 'name', 'value_type', 'input_type', 'created_at')
    search_fields = ('service__service_name', 'name', 'description')
    list_filter = ('value_type', 'created_at', "service__service_name")
    readonly_fields = ('created_at',)
    fieldsets = (
        (None, {
            'fields': ('service', 'name', 'value_type', 'input_type', 'value', 'description', 'meta_info')
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

@admin.register(AppConfig)
class AppConfigAdmin(ModelAdmin):
    list_display = ("is_configured", "created_at")
    
    
@admin.register(DataSource)
class DataSourceAdmin(ModelAdmin):
    list_display = ("name", "interface", "is_available", "last_detected")
    
@admin.register(DataAcquisitionConfig)
class DataAcquisitionConfigAdmin(ModelAdmin):
    list_display = ("selected_source", )
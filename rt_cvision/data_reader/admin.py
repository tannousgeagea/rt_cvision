import os
from django.contrib import admin
from django.utils.html import format_html
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import (
    Image,
    ImageTag
)

from impurity.models import Impurity

class ImpurityInline(TabularInline):
    """Inline admin for Impurity model within Image admin"""
    model = Impurity
    extra = 0  # Number of empty forms to display
    fields = [
        'object_uid', 
        'timestamp', 
        'confidence_score', 
        'class_id', 
        'object_length',
        'object_coordinates',
        'is_processed',
        'meta_info'
    ]
    readonly_fields = ['created_at']
    
    # Optional: Add custom styling or behavior
    classes = ['collapse']  # Makes the inline collapsible
    
    def get_queryset(self, request):
        """Optimize queryset to reduce database queries"""
        return super().get_queryset(request).select_related('image')

# Alternative: Use StackedInline for a different layout
class ImpurityStackedInline(StackedInline):
    """Stacked inline version - better for fewer items with more fields"""
    model = Impurity
    extra = 0
    fields = [
        ('object_uid', 'class_id'),
        ('timestamp', 'confidence_score'),
        ('object_length', 'is_processed'),
        'object_coordinates',
        'meta_info'
    ]
    readonly_fields = ['created_at']

class ImageTagInline(TabularInline):
    model = ImageTag
    extra = 1
    autocomplete_fields = ['tag', 'tagged_by']

@admin.register(Image)
class ImageAdmin(ModelAdmin):
    list_display = ('image_id', 'image_thumbnail', 'file_info', 'timestamp', 'is_processed', 'impurity_count')
    list_filter = ('is_processed', 'source')
    search_fields = ('image_id', 'image_name')
    inlines = [ImpurityInline, ImageTagInline]

    def impurity_count(self, obj):
        """Display count of impurities for each image"""
        count = obj.impurities.count()
        if count > 0:
            return format_html(
                '<span style="color: red; font-weight: bold;">{}</span>', 
                count
            )
        return count

    def formatted_image_name(self, obj):
        """Format image name with icon and styling"""
        if not obj.image_name:
            return format_html(
                '<span style="color: #999; font-style: italic;">No name</span>'
            )
        
        # Add file extension icon
        name = obj.image_name
        if '.' in name:
            ext = name.split('.')[-1].lower()
            if ext in ['jpg', 'jpeg']:
                icon = '🖼️'
            elif ext == 'png':
                icon = '🎨'
            elif ext == 'gif':
                icon = '🎞️'
            else:
                icon = '📄'
        else:
            icon = '📄'
            
        return format_html(
            '{} <strong>{}</strong>',
            icon,
            name
        )
    
    formatted_image_name.short_description = 'Image Name'
    formatted_image_name.admin_order_field = 'image_name'
    
    def image_thumbnail(self, obj):
        """Display small thumbnail of the image"""
        if obj.image_file:
            return format_html(
                '<img src="{}" style="width: 50px; height: 50px; object-fit: cover; border-radius: 4px; border: 1px solid #ddd;" title="{}"/>',
                obj.image_file.url,
                obj.image_name or 'Image'
            )
        return format_html('<span style="color: #999;">No image</span>')
    
    image_thumbnail.short_description = 'Preview'
    
    def image_preview(self, obj):
        """Full size image preview for detail view"""
        if obj.image_file:
            return format_html(
                '''
                <div style="max-width: 400px; margin: 10px 0;">
                    <img src="{}" style="max-width: 100%; height: auto; border: 1px solid #ddd; border-radius: 4px;" />
                    <p style="margin-top: 5px; font-size: 12px; color: #666;">
                        <strong>File:</strong> {} <br>
                        <strong>URL:</strong> <a href="{}" target="_blank">{}</a>
                    </p>
                </div>
                ''',
                obj.image_file.url,
                os.path.basename(obj.image_file.name),
                obj.image_file.url,
                obj.image_file.url
            )
        return format_html('<span style="color: #999;">No image uploaded</span>')
    
    image_preview.short_description = 'Image Preview'
    
    def file_info(self, obj):
        """Display formatted file information"""
        if not obj.image_file:
            return format_html('<span style="color: #999;">No file</span>')
        
        # Get file size
        try:
            file_size = obj.image_file.size
            if file_size:
                if file_size < 1024:
                    size_str = f"{file_size} B"
                elif file_size < 1024 * 1024:
                    size_str = f"{file_size / 1024:.1f} KB"
                else:
                    size_str = f"{file_size / (1024 * 1024):.1f} MB"
            else:
                size_str = "Unknown"
        except (OSError, ValueError):
            size_str = "Unknown"
        
        # Get filename
        filename = os.path.basename(obj.image_file.name)
        
        # Format display
        return format_html(
            '''
            <div style="font-size: 12px;">
                <div><strong>📁 {}</strong></div>
                <div style="color: #666;">📏 {}</div>
                {}
            </div>
            ''',
            filename,
            size_str,
            format_html('<div style="color: #666;">🖼️ {}×{}</div>', obj.width, obj.height) 
            if obj.width and obj.height else ''
        )
    
    file_info.short_description = 'File Info'

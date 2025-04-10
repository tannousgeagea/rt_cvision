import os
import zipfile
import io
from django.http import HttpResponse
from django.contrib import admin
from unfold.admin import ModelAdmin, TabularInline, StackedInline
from .models import (
    Image
)

@admin.action(description="Download selected images as a ZIP")
def download_selected_images(modeladmin, request, queryset):
    """
    Custom admin action to bundle the selected Image objects into a ZIP file
    and send it as a download to the browser.
    """
    # Create an in-memory buffer to build the ZIP file
    buffer = io.BytesIO()

    with zipfile.ZipFile(buffer, 'w') as zip_file:
        for image_obj in queryset:
            # Ensure the ImageField actually has a file
            if image_obj.image_file:
                file_path = image_obj.image_file.path  # local filesystem path
                # The filename to appear in the ZIP
                arcname = os.path.basename(file_path)
                zip_file.write(file_path, arcname=arcname)

    # Move the buffer cursor to the beginning
    buffer.seek(0)

    # Build the HTTP response
    response = HttpResponse(buffer.read(), content_type='application/zip')
    response['Content-Disposition'] = 'attachment; filename="images.zip"'
    return response

@admin.register(Image)
class ImageAdmin(ModelAdmin):
    list_display = ('image_id', 'image_name', 'image_file')
    actions = [download_selected_images]
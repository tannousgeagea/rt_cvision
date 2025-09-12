import os
import time
import django
django.setup()
import logging
from asgiref.sync import sync_to_async
from fastapi import APIRouter
from fastapi.routing import APIRoute
from fastapi import FastAPI, HTTPException, Query
from pydantic import BaseModel, Field
from typing import List, Optional, Callable
from fastapi import Request, Response
from datetime import datetime
from django.db.models import Q
from data_reader.models import Image
from impurity.models import Impurity
from fastapi import FastAPI, HTTPException, status
from fastapi.responses import JSONResponse
from django.db import transaction
from django.utils import timezone
from django.db.models import Q
from django.core.files.storage import default_storage
from django.core.exceptions import ObjectDoesNotExist
from data_reader.models import Image
from configure.routers.deletion.schemas import DeleteResponse, ImageDeleteRequest, validate_filters

logger = logging.getLogger(__name__)

class TimedRoute(APIRoute):
    def get_route_handler(self) -> Callable:
        original_route_handler = super().get_route_handler()
        async def custom_route_handler(request: Request) -> Response:
            before = time.time()
            response: Response = await original_route_handler(request)
            duration = time.time() - before
            response.headers["X-Response-Time"] = str(duration)
            print(f"route duration: {duration}")
            print(f"route response: {response}")
            print(f"route response headers: {response.headers}")
            return response

        return custom_route_handler

router = APIRouter(
    route_class=TimedRoute
)

@router.delete("/images/delete", response_model=DeleteResponse)
async def delete_images(delete_request: ImageDeleteRequest):
    """
    Delete images based on various filter criteria.
    
    Supports deletion by:
    - Single image ID or list of image IDs
    - Date/time ranges (created_at and timestamp)
    - Source, processing status, sensorbox
    - Expired images only
    
    Safety features:
    - Requires at least one filter to prevent accidental deletion of all images
    - Can force delete even with related impurities
    """
    try:
        # Validate at least one filter is provided
        if not validate_filters(delete_request.model_dump()):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="At least one filter parameter must be provided to prevent accidental deletion of all images."
            )
        
        # Build Django Q objects for filtering
        filters = Q()
        
        # Single image ID
        if delete_request.image_id:
            filters &= Q(image_id=delete_request.image_id)
        
        # Multiple image IDs
        if delete_request.image_ids:
            filters &= Q(image_id__in=delete_request.image_ids)
        
        # Created datetime range
        if delete_request.created_after:
            filters &= Q(created_at__gte=delete_request.created_after)
        if delete_request.created_before:
            filters &= Q(created_at__lte=delete_request.created_before)
        
        # Timestamp range
        if delete_request.timestamp_after:
            filters &= Q(timestamp__gte=delete_request.timestamp_after)
        if delete_request.timestamp_before:
            filters &= Q(timestamp__lte=delete_request.timestamp_before)
        
        # Additional filters
        if delete_request.source:
            filters &= Q(source=delete_request.source)
        if delete_request.is_processed is not None:
            filters &= Q(is_processed=delete_request.is_processed)
        if delete_request.sensorbox_id:
            filters &= Q(sensorbox_id=delete_request.sensorbox_id)
        
        # Expired images only
        if delete_request.delete_expired_only:
            current_time = timezone.now()
            filters &= Q(expires_at__isnull=False, expires_at__lte=current_time)

        # Wrap the database operations in sync_to_async
        @sync_to_async
        def perform_delete_operation():
            with transaction.atomic():
                # Get images to delete
                images_to_delete = Image.objects.filter(filters)
                
                if not images_to_delete.exists():
                    return {
                        "success": True,
                        "deleted_count": 0,
                        "message": "No images found matching the specified criteria",
                        "deleted_ids": []
                    }
                
                # Check for related impurities if not force deleting
                if not delete_request.force_delete:
                    images_with_impurities = []
                    for image in images_to_delete:
                        if image.impurities.exists():
                            images_with_impurities.append(image.image_id)
                    
                    if images_with_impurities:
                        raise HTTPException(
                            status_code=status.HTTP_400_BAD_REQUEST,
                            detail={
                                "error": "Cannot delete images with related impurities",
                                "images_with_impurities": images_with_impurities,
                                "suggestion": "Use force_delete=true to delete anyway, or delete impurities first"
                            }
                        )
                    
                
                # Force delete: first delete all related impurities
                impurities_to_delete = Impurity.objects.filter(image__in=images_to_delete)
                impurities_count = impurities_to_delete.count()
                if impurities_count > 0:
                    impurities_to_delete.delete()
                    logger.info(f"Force delete: Deleted {impurities_count} related impurities before deleting images")
                
                
                # Store image IDs for response
                deleted_image_ids = list(images_to_delete.values_list('image_id', flat=True)) 
                image_files = []
                
                # Collect image files before deletion
                for image in images_to_delete:
                    if image.image_file and image.image_file.name:
                        image_files.append(image.image_file.name)
                

                # Perform deletion
                deleted_count, _ = images_to_delete.delete()

                # Delete physical files using Django storage
                files_deleted = 0
                files_not_found = 0
                for file_name in image_files:
                    try:
                        if default_storage.exists(file_name):
                            default_storage.delete(file_name)
                            files_deleted += 1
                            logger.debug(f"Deleted file: {file_name}")
                        else:
                            files_not_found += 1
                            logger.warning(f"File not found in storage: {file_name}")
                    except Exception as e:
                        logger.error(f"Failed to delete file {file_name} from storage: {str(e)}")
                        # Don't raise exception for file deletion failures
                        # The database records are already deleted

                logger.info(f"Deleted {deleted_count} images: {deleted_image_ids}")
                logger.info(f"Deleted {files_deleted} image files, {files_not_found} files not found")

                response_message = f"Successfully deleted {deleted_count} images and {files_deleted} files"
                if delete_request.force_delete and impurities_count > 0:
                    response_message += f" and {impurities_count} related impurities"
                if files_not_found > 0:
                    response_message += f" ({files_not_found} files were already missing)"
                    
                return {
                    "success": True,
                    "deleted_count": deleted_count,
                    "message": response_message,
                    "deleted_ids": deleted_image_ids
                }
        
        # Execute the database operations asynchronously
        result = await perform_delete_operation()
        
        return DeleteResponse(**result)
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting images: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}"
        )

# Convenience endpoints for single item deletion
@router.delete("/images/{image_id}")
async def delete_single_image(image_id: str, force_delete: bool = False):
    """Quick endpoint to delete a single image by ID"""
    delete_request = ImageDeleteRequest(
        image_id=image_id,
        force_delete=force_delete
    )
    return await delete_images(delete_request)

@router.post("/cleanup/expired-images")
async def cleanup_expired_images():
    """Convenience endpoint to clean up expired images"""
    delete_request = ImageDeleteRequest(delete_expired_only=True, force_delete=True)
    return await delete_images(delete_request)

# Bulk operations endpoints
@router.delete("/images/bulk")
async def bulk_delete_images(image_ids: List[str], force_delete: bool = False):
    """Bulk delete multiple images by IDs"""
    delete_request = ImageDeleteRequest(
        image_ids=image_ids,
        force_delete=force_delete
    )
    return await delete_images(delete_request)
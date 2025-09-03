import os
import time
import django
django.setup()
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
from django.core.exceptions import ObjectDoesNotExist
from data_reader.models import Image
from configure.routers.deletion.schemas import DeleteResponse, ImageDeleteRequest, validate_filters

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
        
        with transaction.atomic():
            # Get images to delete
            images_to_delete = Image.objects.filter(filters)
            
            if not images_to_delete.exists():
                return DeleteResponse(
                    success=True,
                    deleted_count=0,
                    message="No images found matching the specified criteria",
                    deleted_ids=[]
                )
            
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
            
            # Store image IDs for response
            deleted_image_ids = list(images_to_delete.values_list('image_id', flat=True))
            
            # Perform deletion
            deleted_count, _ = images_to_delete.delete()
            
            logger.info(f"Deleted {deleted_count} images: {deleted_image_ids}")
            
            return DeleteResponse(
                success=True,
                deleted_count=deleted_count,
                message=f"Successfully deleted {deleted_count} images",
                deleted_ids=deleted_image_ids
            )
    
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
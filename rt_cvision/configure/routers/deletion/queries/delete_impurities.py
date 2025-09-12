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
from django.core.exceptions import ObjectDoesNotExist
from data_reader.models import Image
from configure.routers.deletion.schemas import DeleteResponse, ImpurityDeleteRequest, validate_filters

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

@router.delete("/impurities/delete", response_model=DeleteResponse)
async def delete_impurities(delete_request: ImpurityDeleteRequest):
    """
    Delete impurities based on various filter criteria.
    
    Supports deletion by:
    - Single impurity ID or list of impurity IDs
    - Image ID(s) - deletes all impurities from specified images
    - Object properties (object_uid, class_id)
    - Confidence score range
    - Date/time ranges
    - Processing status
    """
    try:
        # Validate at least one filter is provided
        if not validate_filters(delete_request.dict()):
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="At least one filter parameter must be provided to prevent accidental deletion of all impurities."
            )
        
        # Build Django Q objects for filtering
        filters = Q()
        
        # Single impurity ID
        if delete_request.impurity_id:
            filters &= Q(id=delete_request.impurity_id)
        
        # Multiple impurity IDs
        if delete_request.impurity_ids:
            filters &= Q(id__in=delete_request.impurity_ids)
        
        # Filter by image ID
        if delete_request.image_id:
            filters &= Q(image__image_id=delete_request.image_id)
        
        # Filter by multiple image IDs
        if delete_request.image_ids:
            filters &= Q(image__image_id__in=delete_request.image_ids)
        
        # Object properties
        if delete_request.object_uid:
            filters &= Q(object_uid=delete_request.object_uid)
        if delete_request.class_id is not None:
            filters &= Q(class_id=delete_request.class_id)
        
        # Confidence score range
        if delete_request.confidence_min is not None:
            filters &= Q(confidence_score__gte=delete_request.confidence_min)
        if delete_request.confidence_max is not None:
            filters &= Q(confidence_score__lte=delete_request.confidence_max)
        
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
        
        # Processing status
        if delete_request.is_processed is not None:
            filters &= Q(is_processed=delete_request.is_processed)

        # Wrap the database operations in sync_to_async
        @sync_to_async
        def perform_delete_operation():
            with transaction.atomic():
                # Get impurities to delete
                impurities_to_delete = Impurity.objects.filter(filters)
                
                if not impurities_to_delete.exists():
                    return {
                        "success": True,
                        "deleted_count": 0,
                        "message": "No impurities found matching the specified criteria",
                        "deleted_ids": []
                    }
                
                # Store impurity IDs for response
                deleted_impurity_ids = list(impurities_to_delete.values_list('id', flat=True))
                deleted_impurity_ids = [str(id) for id in deleted_impurity_ids]
                
                # Perform deletion
                deleted_count, _ = impurities_to_delete.delete()
                
                logger.info(f"Deleted {deleted_count} impurities: {deleted_impurity_ids}")
                
                return {
                    "success": True,
                    "deleted_count": deleted_count,
                    "message": f"Successfully deleted {deleted_count} impurities",
                    "deleted_ids": deleted_impurity_ids
                }
        
        # Execute the database operations asynchronously
        result = await perform_delete_operation()
        
        return DeleteResponse(**result)
    
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting impurities: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}"
        )


@router.delete("/impurities/{impurity_id}")
async def delete_single_impurity(impurity_id: int):
    """Quick endpoint to delete a single impurity by ID"""
    delete_request = ImpurityDeleteRequest(impurity_id=impurity_id)
    return await delete_impurities(delete_request)

@router.delete("/impurities/bulk")
async def bulk_delete_impurities(impurity_ids: List[int]):
    """Bulk delete multiple impurities by IDs"""
    delete_request = ImpurityDeleteRequest(impurity_ids=impurity_ids)
    return await delete_impurities(delete_request)

@router.delete("/impurities/by-image/{image_id}")
async def delete_impurities_by_image(image_id: str):
    """Delete all impurities associated with a specific image"""
    delete_request = ImpurityDeleteRequest(image_id=image_id)
    return await delete_impurities(delete_request)
# schemas.py
from pydantic import BaseModel, Field, validator
from typing import List, Optional
from datetime import datetime

class DeleteResponse(BaseModel):
    success: bool
    deleted_count: int
    message: str
    deleted_ids: Optional[List[str]] = None

class ImageDeleteRequest(BaseModel):
    # Single image ID
    image_id: Optional[str] = Field(None, description="Single image ID to delete")
    
    # List of image IDs
    image_ids: Optional[List[str]] = Field(None, description="List of image IDs to delete")
    
    # DateTime range filters
    created_after: Optional[datetime] = Field(None, description="Delete images created after this datetime")
    created_before: Optional[datetime] = Field(None, description="Delete images created before this datetime")
    
    # Timestamp range filters (for capture time)
    timestamp_after: Optional[datetime] = Field(None, description="Delete images with timestamp after this datetime")
    timestamp_before: Optional[datetime] = Field(None, description="Delete images with timestamp before this datetime")
    
    # Additional filters
    source: Optional[str] = Field(None, description="Delete images from specific source")
    is_processed: Optional[bool] = Field(None, description="Filter by processing status")
    sensorbox_id: Optional[int] = Field(None, description="Delete images from specific sensorbox")
    
    # Safety options
    force_delete: bool = Field(False, description="Force delete even if there are related impurities")
    delete_expired_only: bool = Field(False, description="Only delete expired images")

class ImpurityDeleteRequest(BaseModel):
    # Single impurity ID
    impurity_id: Optional[int] = Field(None, description="Single impurity ID to delete")
    
    # List of impurity IDs
    impurity_ids: Optional[List[int]] = Field(None, description="List of impurity IDs to delete")
    
    # Filter by image
    image_id: Optional[str] = Field(None, description="Delete all impurities from specific image")
    image_ids: Optional[List[str]] = Field(None, description="Delete all impurities from list of images")
    
    # Filter by object properties
    object_uid: Optional[str] = Field(None, description="Delete impurity with specific object UID")
    class_id: Optional[int] = Field(None, description="Delete impurities with specific class ID")
    
    # Confidence score range
    confidence_min: Optional[float] = Field(None, ge=0.0, le=1.0, description="Minimum confidence score")
    confidence_max: Optional[float] = Field(None, ge=0.0, le=1.0, description="Maximum confidence score")
    
    # DateTime range filters
    created_after: Optional[datetime] = Field(None, description="Delete impurities created after this datetime")
    created_before: Optional[datetime] = Field(None, description="Delete impurities created before this datetime")
    timestamp_after: Optional[datetime] = Field(None, description="Delete impurities with timestamp after this datetime")
    timestamp_before: Optional[datetime] = Field(None, description="Delete impurities with timestamp before this datetime")
    
    # Processing status
    is_processed: Optional[bool] = Field(None, description="Filter by processing status")

def validate_filters(request_data: dict) -> bool:
    """Validate that at least one filter is provided"""
    filter_fields = [
        'image_id', 'image_ids', 'created_after', 'created_before',
        'timestamp_after', 'timestamp_before', 'source', 'is_processed',
        'sensorbox_id', 'delete_expired_only', 'impurity_id', 'impurity_ids',
        'object_uid', 'class_id', 'confidence_min', 'confidence_max'
    ]
    
    return any(request_data.get(field) for field in filter_fields if request_data.get(field) is not None)

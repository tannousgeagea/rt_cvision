
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
from django.core.exceptions import ObjectDoesNotExist

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

class ImpurityOut(BaseModel):
    object_uid: str
    confidence_score: float
    class_id: int
    object_coordinates: List[float]
    timestamp: datetime
    object_length: Optional[float] = None
    meta_info: Optional[dict] = None

    class Config:
        from_attributes = True


class ImageOut(BaseModel):
    image_id: str
    image_name: str
    src: str
    timestamp: datetime
    image_format: Optional[str]
    image_size: Optional[int]
    width: Optional[int]
    height: Optional[int]
    source: Optional[str]
    meta_info: Optional[dict]
    impurities: List[ImpurityOut]

    class Config:
        from_attributes = True

@router.api_route("/images", methods=["GET"])
def list_images(
    tag: Optional[str] = Query(None, description="Filter by single tag name"),
    tags: Optional[List[str]] = Query(None, description="Filter by multiple tag names"),
    from_date: Optional[datetime] = Query(None, description="Filter from timestamp"),
    to_date: Optional[datetime] = Query(None, description="Filter to timestamp")
) -> List[ImageOut]:
    images = Image.objects.prefetch_related("impurities").order_by('-created_at')

    # --- Filter by time range ---
    if from_date:
        images = images.filter(timestamp__gte=from_date)
    if to_date:
        images = images.filter(timestamp__lte=to_date)
    
    if tag:
        images = images.filter(impurities__tags__name=tag).distinct()
    if tags:
        images = images.filter(impurities__tags__name__in=tags).distinct()
        
    return [
        ImageOut(
            image_id=img.image_id,
            image_name=img.image_name,
            src=img.image_file.url,
            timestamp=img.timestamp,
            image_format=img.image_format,
            image_size=img.image_size,
            width=img.width,
            height=img.height,
            source=img.source,
            meta_info=img.meta_info,
            impurities=[
                ImpurityOut(
                    object_uid=i.object_uid,
                    confidence_score=i.confidence_score,
                    class_id=i.class_id,
                    object_coordinates=i.object_coordinates,
                    timestamp=i.timestamp,
                    meta_info=i.meta_info,
                    object_length=i.object_length,
                )
                for i in img.impurities.all()
            ]
        )
        for img in images
    ]

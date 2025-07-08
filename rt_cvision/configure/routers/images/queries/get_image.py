import os
import time
import django
django.setup()
from fastapi import APIRouter
from fastapi.routing import APIRoute
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field
from typing import List, Optional, Callable
from fastapi import Request, Response
from datetime import datetime
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
    meta_info: Optional[dict] = None

    class Config:
        from_attributes = True


class ImageOut(BaseModel):
    image_id: str
    image_name: str
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

@router.api_route("/images/{image_id}/", response_model=ImageOut)
def get_image(image_id: str):
    try:
        image = Image.objects.prefetch_related("impurities").get(image_id=image_id)
    except ObjectDoesNotExist:
        raise HTTPException(status_code=404, detail="Image not found")

    return ImageOut(
        image_id=image.image_id,
        image_name=image.image_name,
        timestamp=image.timestamp,
        image_format=image.image_format,
        image_size=image.image_size,
        width=image.width,
        height=image.height,
        source=image.source,
        meta_info=image.meta_info,
        impurities=[
            ImpurityOut(
                object_uid=i.object_uid,
                confidence_score=i.confidence_score,
                class_id=i.class_id,
                object_coordinates=i.object_coordinates,
                timestamp=i.timestamp,
                meta_info=i.meta_info
            )
            for i in image.impurities.all()
        ]
    )

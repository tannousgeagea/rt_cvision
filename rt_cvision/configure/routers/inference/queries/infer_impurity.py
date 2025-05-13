
import os
import cv2
import math
import uuid
import time
import django
import shutil
django.setup()
import numpy as np
from datetime import datetime, timedelta
from datetime import date, timezone
from typing import Callable, Optional, Dict, AnyStr, Any
from fastapi import Request
from fastapi import Response
from fastapi import APIRouter
from fastapi import Depends, Form, Body, Query
from fastapi import HTTPException
from fastapi.responses import JSONResponse
from fastapi.routing import APIRoute
from fastapi import status
from fastapi import File, UploadFile
from pathlib import Path
from pydantic import BaseModel
from configure.models import Service, ServiceParams
from configure.ml_model_loader import get_model
from common_utils.model.base import BaseModels as MLModel

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
    route_class=TimedRoute,
)


params = {}
service = Service.objects.filter(service_name="impurity").first()
service_params = ServiceParams.objects.filter(service=service, is_active=True)
for param in service_params:
    params[param.name] = param.value

ai_model = get_model(
    weights=params.get("weights"),
    mlflow=params.get('mlflow', {}).get('active', False),
    task="impurity"
)

@router.api_route(
    "/infer/impurity", methods=["POST"],
)
async def infer(image: UploadFile = File(...), conf:float=0.25):
    image_bytes = await image.read()
    nparr = np.frombuffer(image_bytes, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

    start_time = time.time()
    detections = ai_model.classify_one(
        image=img,
        conf=conf,
        mode="detect",
    )

    end_time = time.time()

    predictions = [
        {
            "x": int(xyxy[0]),
            "y": int(xyxy[1]),
            "width": int(xyxy[2] - xyxy[0]),
            "height": int(xyxy[3] - xyxy[1]), 
            "confidence": float(detections.confidence[i]),
            "class_id": int(detections.class_id[i]),
            "xyxyn": detections.xyxyn[i].tolist(),
            "id": str(i),
        } for i, xyxy in enumerate(detections.xyxy.tolist())
    ]

    return {
        "predictions": predictions,
        "height": img.shape[0],
        "width": img.shape[1],
        "key": image.filename,
        "inference_time": round(end_time - start_time, 3)
        }
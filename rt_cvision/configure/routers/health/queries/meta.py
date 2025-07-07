
import os
import time
import psutil
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
from tenants.models import Tenant
from django.utils.timezone import now
from common_utils.health.metadata import get_microservices
from django.core.exceptions import ObjectDoesNotExist
from common_utils.health.runtime import get_supervisord_uptime
from common_utils.health.metrics import get_metrics
from common_utils.health.health_score import calculate_health_score

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


@router.api_route(
    "/metadata", methods=["GET"]
    )
def metadata():
    tenant = Tenant.objects.all().first()
    if not tenant:
        raise HTTPException(
            status_code=404, detail=f"Service not yet configured"
        )
    
    errors = get_metrics()["errors"]
    requests = get_metrics()["requests"]
    health_score = calculate_health_score(errors, requests, health_ok=True)
    return {
        "id": f"rtcvision-{tenant.tenant_id}",
        "name": f"RTCVision {tenant.tenant_name}",
        "description": f"Real-Time Computer Vision @ {tenant.tenant_name} in {tenant.location}",
        "status": "active",
        "version": os.getenv("RTCVISION_VERSION", "1.0.0"),
        "environment": os.getenv("ENVIRONMENT", "production"),
        "uptime": get_supervisord_uptime(),
        "lastDeployed": os.getenv("LAST_DEPLOYED", now().isoformat()),
        "cpu": psutil.cpu_percent(interval=0.1),
        "memory": psutil.virtual_memory().percent,
        "healthScore": health_score,
        "requests": requests,
        "errors": errors,
        "tenantId": tenant.tenant_id,
        "microservices": get_microservices()
    }


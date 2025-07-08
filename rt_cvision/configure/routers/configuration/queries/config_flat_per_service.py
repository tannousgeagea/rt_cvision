import time
import json
import os
from pydantic import BaseModel
from typing import Optional, List, Any, Callable
from fastapi import APIRouter, HTTPException
from fastapi.routing import APIRoute
from fastapi.responses import JSONResponse
from fastapi import Response, Request
from fastapi import Query
from fastapi.responses import PlainTextResponse
from common_utils.caching import get_cache_backend
from common_utils.caching.utils import make_cache_key
from configure.models import (
    Service,
    ServiceConfigGroup,
    
)

cache = get_cache_backend()

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

@router.get("/services/{service_id}/config/flat", response_class=JSONResponse)
def get_service_config_flat(
    service_id: str,
    include_inactive: bool = Query(False, description="Include inactive parameters"),
    format: str = Query("json", enum=["json", "env"], description="Response format: json or env")
    ):
    cache_key = make_cache_key("service_config", {
        "service_id": service_id,
        "include_inactive": include_inactive,
        "format": format
    })
    
    cached_result = cache.get(cache_key)
    if cached_result:
        return PlainTextResponse(cached_result) if format == "env" else JSONResponse(content=json.loads(cached_result))

    try:
        service = Service.objects.get(service_id=service_id)
    except Service.DoesNotExist:
        raise HTTPException(status_code=404, detail="Service not found")
    
    config = {}
    groups = ServiceConfigGroup.objects.filter(service=service, is_active=True).prefetch_related(
        'fields__definition'
    )

    for group in groups:
        fields = group.fields.all()
        if not include_inactive:
            fields = fields.filter(is_active=True)

        for field_instance in fields:
            definition = field_instance.definition
            value = field_instance.value

            # If not set, fall back to default
            if value is None:
                value = definition.default_value

            config[definition.field_id] = value

    if format == "env":
        lines = []
        for k, v in config.items():
            if isinstance(v, bool):
                v = str(v).lower()
            elif isinstance(v, (list, dict)):
                v = json.dumps(v)
            lines.append(f"{k.upper()}={v}")
        env_text = "\n".join(lines)
        cache.set(cache_key, env_text, ttl=60)
        return PlainTextResponse(env_text)

    cache.set(cache_key, json.dumps(config), ttl=60)
    return JSONResponse(content=config)
import time
import json
from pydantic import BaseModel
from typing import Optional, List, Any, Callable
from fastapi import APIRouter, HTTPException
from fastapi.routing import APIRoute
from fastapi import Response, Request
from fastapi import Query
from common_utils.caching import get_cache_backend
from common_utils.caching.utils import make_cache_key
from configure.models import (
    Service,
    ServiceConfigGroup,
    
)

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

class ConfigField(BaseModel):
    field_id: str
    label: str
    value: Any
    value_type: str

class ConfigGroup(BaseModel):
    group: str
    fields: List[ConfigField]

class ServiceConfigResponse(BaseModel):
    service_id: str
    service_name: str
    config: List[ConfigGroup]

@router.get("/services/{service_id}/config", response_model=ServiceConfigResponse)
def get_service_config(
    service_id: str,
    include_inactive: bool = Query(False, description="Include inactive parameters")
    ):
    cache = get_cache_backend()
    cache_key = make_cache_key("service_config", {
        "service_id": service_id,
        "include_inactive": include_inactive,
    })
    
    print(cache_key)
    cached_result = cache.get(cache_key)
    if cached_result:
        return json.loads(cached_result)

    try:
        service = Service.objects.get(service_id=service_id)
    except Service.DoesNotExist:
        raise HTTPException(status_code=404, detail="Service not found")
    
    config = []

    groups = ServiceConfigGroup.objects.filter(service=service, is_active=True).order_by('order').prefetch_related(
        'fields__definition', 'fields__definition__value_type'
    )

    for group in groups:
        fields = []
        all_fields = group.fields.all()
        if not include_inactive:
            all_fields = all_fields.filter(is_active=True)

        for field_instance in all_fields:
            definition = field_instance.definition
            value_type = definition.value_type.name
            fields.append({
                "field_id": definition.field_id,
                "label": definition.label,
                "value": field_instance.value or definition.default_value,
                "value_type": value_type
            })

        config.append({
            "group": group.name,
            "fields": fields
        })

    result = {
        "service_id": service.service_id,
        "service_name": service.service_name,
        "config": config
    }

    cache.set(cache_key, json.dumps(result), ttl=60)
    return result
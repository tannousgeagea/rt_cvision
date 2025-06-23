import json
import time
from fastapi import APIRouter, HTTPException
from common_utils.caching import get_cache_backend
from common_utils.caching.utils import make_cache_key
from pydantic import BaseModel
from typing import Optional, List, Callable
from fastapi.routing import APIRoute
from fastapi import Response, Request

from tenants.models import (
    Tenant, 
    TenantStorageSettings, 
    EntityType, 
    PlantEntity, 
    SensorBox,
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


class SensorBoxOut(BaseModel):
    id: int
    name: str
    location: str

class PlantEntityOut(BaseModel):
    id: int
    uid: str
    description: str
    entity_type: str

class TenantConfigResponse(BaseModel):
    tenant_id: str
    tenant_name: str
    domain: str
    default_language: Optional[str]
    storage_provider: Optional[str]
    storage_account: Optional[str]
    entity: Optional[PlantEntityOut]
    sensor_box: Optional[SensorBoxOut]

@router.get("/tenant/config", response_model=TenantConfigResponse)
def get_tenant_config():
    cache = get_cache_backend()
    cache_key = make_cache_key("tenant_config", {})

    cached = cache.get(cache_key)
    if cached:
        return json.loads(cached)

    try:
        tenant = Tenant.objects.filter(is_active=True).first()
    except Tenant.DoesNotExist:
        raise HTTPException(status_code=404, detail="Tenant not found")

    storage = getattr(tenant, "storage_settings", None)
    entity_type = EntityType.objects.filter(tenant=tenant).first()
    entity = PlantEntity.objects.filter(entity_type=entity_type).first()
    sensor_box = SensorBox.objects.filter(plant_entity=entity).first()

    result = TenantConfigResponse(
        tenant_id=tenant.tenant_id,
        tenant_name=tenant.tenant_name,
        domain=tenant.domain,
        default_language=tenant.default_language,
        storage_provider=storage.provider_name if storage else None,
        storage_account=storage.account_name if storage else None,
        entity=PlantEntityOut(
            id=entity.id,
            uid=entity.entity_uid,
            description=entity.description,
            entity_type=entity_type.entity_type
        ) if entity else None,
        sensor_box=SensorBoxOut(
            id=sensor_box.id,
            name=sensor_box.sensor_box_name,
            location=sensor_box.sensor_box_location
        ) if sensor_box else None
    )

    # Cache result (as JSON string)
    cache.set(cache_key, result.model_dump_json(), ttl=60)
    return result

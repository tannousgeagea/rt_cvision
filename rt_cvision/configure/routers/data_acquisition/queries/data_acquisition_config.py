import json
import time
from datetime import datetime
from fastapi import APIRouter, HTTPException
from common_utils.caching import get_cache_backend
from common_utils.caching.utils import make_cache_key
from pydantic import BaseModel
from typing import Optional, List, Callable
from fastapi.routing import APIRoute
from fastapi import Response, Request
from common_utils.caching import get_cache_backend
from common_utils.caching.utils import make_cache_key
from configure.models import DataSource, DataAcquisitionConfig

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

class DataSourceOut(BaseModel):
    id: int
    name: str
    interface: str
    is_available: bool
    last_detected: datetime

class DataAcquisitionConfigOut(BaseModel):
    selected_source: Optional[DataSourceOut]

@router.get("/data-acquisition/config", response_model=DataAcquisitionConfigOut)
def get_data_acquisition_config():
    cache = get_cache_backend()
    cache_key = make_cache_key("data_acquisition_config", {})

    cached = cache.get(cache_key)
    if cached:
        return json.loads(cached)
    
    selected = DataAcquisitionConfig.get_selected_source()
    source = None
    if selected:
        source = DataSourceOut(
            id=selected.id,
            name=selected.name,
            interface=selected.interface,
            is_available=selected.is_available,
            last_detected=selected.last_detected
        )

    result = DataAcquisitionConfigOut(selected_source=source)
    cache.set(cache_key, result.model_dump_json(), ttl=30)
    return result

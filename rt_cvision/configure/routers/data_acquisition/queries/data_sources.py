

import json
import time
from datetime import datetime
from fastapi import APIRouter, HTTPException
from common_utils.caching import get_cache_backend
from common_utils.caching.utils import make_cache_key
from pydantic import BaseModel
from typing import Optional, List, Callable
from fastapi.routing import APIRoute
from fastapi import Response, Request, Query
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

@router.get("/data-acquisition/sources", response_model=List[DataSourceOut])
def get_data_sources(interface: Optional[str] = Query(None, description="Filter by interface")):
    cache = get_cache_backend()
    cache_key = make_cache_key("data_source_list", {"interface": interface or "all"})

    cached = cache.get(cache_key)
    if cached:
        return [DataSourceOut(**item) for item in json.loads(cached)]
    
    if interface:
        sources = DataSource.get_available_sources(interface)
    else:
        sources = DataSource.objects.filter(is_available=True)

    result = [
        DataSourceOut(
            id=src.id,
            name=src.name,
            interface=src.interface,
            is_available=src.is_available,
            last_detected=src.last_detected
        ) for src in sources
    ]

    cache.set(cache_key, json.dumps([r.model_dump(mode="json") for r in result]), ttl=30)
    return result
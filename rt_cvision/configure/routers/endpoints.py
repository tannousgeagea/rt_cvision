import os
import math
import time
import django
django.setup()

from fastapi import status
from typing import Callable
from fastapi import Request
from fastapi import Response
from fastapi import APIRouter
from django.db import connection
from fastapi import HTTPException
from datetime import date, timezone
from fastapi.routing import APIRoute
from datetime import datetime, timedelta
from fastapi.responses import JSONResponse

from configure.models import Service, ServiceParams

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
    prefix="/api/v1",
    tags=["Service"],
    route_class=TimedRoute,
    responses={404: {"description": "Not found"}},
)

@router.api_route(
    "/services/metadata", methods=["GET"], tags=["Service"]
)
def get_delivery_metadata(response: Response):
    results = {}
    try:
        row = []
        services = Service.objects.all()
        for service in services:
            data = []
            params = ServiceParams.objects.filter(service=service)
            for param in params:
                data.append(
                    {
                        "name": param.name,
                        "value": param.value,
                        "input_type": param.input_type,
                        "description": param.description,
                        "meta_info": param.meta_info,
                    }
                )
            row.append(
                {
                    'service_name': service.service_name,
                    'service_id': service.service_id,
                    'params': data,   
                }
            )
        
        results = {
            'data': row
        }
            
    except HTTPException as e:
        results['error'] = {
            "status_code": "not found",
            "status_description": "Request not Found",
            "detail": f"{e}",
        }
        
        response.status_code = status.HTTP_404_NOT_FOUND
    
    except Exception as e:
        results['error'] = {
            'status_code': 'server-error',
            "status_description": "Internal Server Error",
            "detail": str(e),
        }
        
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
    
    return results


@router.api_route(
    "/services/", methods=["GET"], tags=["Service"]
)
def get_delivery_metadata(response: Response):
    results = {}
    try:
        row = []
        services = Service.objects.all()
        for service in services:
            data = []
            params = ServiceParams.objects.filter(service=service)
            for param in params:
                data.append(
                    {
                        "name": param.name,
                        "value": param.value,
                        "input_type": param.input_type,
                        "description": param.description,
                        "meta_info": param.meta_info,
                    }
                )
            row.append(
                {
                    'service_name': service.service_name,
                    'service_id': service.service_id,
                    'params': data,   
                }
            )
        
        results = {
            'data': row
        }
            
    except HTTPException as e:
        results['error'] = {
            "status_code": "not found",
            "status_description": "Request not Found",
            "detail": f"{e}",
        }
        
        response.status_code = status.HTTP_404_NOT_FOUND
    
    except Exception as e:
        results['error'] = {
            'status_code': 'server-error',
            "status_description": "Internal Server Error",
            "detail": str(e),
        }
        
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
    
    return results
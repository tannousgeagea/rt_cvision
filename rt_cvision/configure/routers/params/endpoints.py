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
from xmlrpc.client import ServerProxy
server = ServerProxy('http://appuser:wasteantadmin@2024@localhost:19001/RPC2')

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
    tags=["ServiceParams"],
    route_class=TimedRoute,
    responses={404: {"description": "Not found"}},
)

@router.api_route(
    "/params", methods=["GET"], tags=["ServiceParams"]
)
def get_service_metadata(response: Response):
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
    "/params/{service_name}", methods=["GET"], tags=["ServiceParams"]
)
def get_service_data_by_service_name(response: Response, service_name:str):
    results = {}
    try:
        data = []

        if not Service.objects.filter(service_name=service_name).exists():
            results['error'] = {
                "status_code": "not found",
                "status_description": f"query service_name {service_name} not found",
                "detail": f"query service_name {service_name} not found; Possible options: {[s.service_name for s in Service.objects.all()]}",
            }
            response.status_code = status.HTTP_400_BAD_REQUEST
            
            return results
            
        service = Service.objects.get(service_name=service_name)
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
        
        results = {
            'data': {
                'service_name': service_name,
                'service_id': service.service_id,
                'params': data,
            }
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
    "/params/{service_name}", methods=["PUT"], tags=["ServiceParams"]
)
def update_service_params(response: Response, service_name:str, params:dict):
    results = {
        'status_code': 'ok',
        'status_description': '',
        'details': []
    }
    try:
        restart = False
        if 'restart' in params:
            restart = params.get('restart')
            params.pop('restart')
            
        time.sleep(1)
        if not Service.objects.filter(service_name=service_name).exists():
            results['error'] = {
                "status_code": "not found",
                "status_description": f"query service_name {service_name} not found",
                "detail": f"query service_name {service_name} not found; Possible options: {[s.service_name for s in Service.objects.all()]}",
            }
            response.status_code = status.HTTP_400_BAD_REQUEST
            
            return results
        
        failed_update = []
        successful_update = []
        
        service = Service.objects.get(service_name=service_name)
        old_params = ServiceParams.objects.filter(service=service)
        for name, value in params.items():
            if not ServiceParams.objects.filter(service=service, name=name).exists():
                failed_update.append(
                    {
                        'name': name,
                        'value': value,
                        'status': 'failed',
                        'reason': f"param name {name} not found. Possible options: {[p.name for p in old_params]}", 
                    }
                )    
                continue
            
            try:
                old_param = ServiceParams.objects.get(service=service, name=name)
                old_param.value = value
                old_param.save()
            except Exception as err:
                failed_update.append(
                    {
                        'name': name,
                        'value': value,
                        'status': 'failed',
                        'reason': f"{err}", 
                    }
                )
                continue
            
            successful_update.append(
                {
                    'name': name,
                    'value': value,
                    'status': 'success',
                }
            )
            
            results['details'].append(successful_update)
            
        if failed_update:
            results['status_code'] = 'partial-success'
            results['status_description'] = f'{len(successful_update)} parameters updated successfully, {len(failed_update)} parameters failed'
            results['details'].extend(failed_update)
        else:
            results['status_description'] = f'{len(successful_update)} parameters updated successfully'
            
        if restart:
            server.supervisor.stopProcessGroup(service_name)
            server.supervisor.startProcessGroup(service_name)
            results['status_description'].extends(f"")

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
import os
import math
import time
import psutil
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
from collections import defaultdict
from fastapi.routing import APIRoute
from datetime import datetime, timedelta
from fastapi.responses import JSONResponse
from common_utils.health.runtime import get_supervisord_uptime, get_supervisord_last_deployed
from common_utils.health.metrics import get_metrics
from common_utils.health.health_score import calculate_health_score


from tenants.models import Tenant

from configure.models import (
    Service, 
    ServiceParams,
    ServiceConfigGroup,
    ServiceConfigFieldInstance,
)
from xmlrpc.client import ServerProxy
server = ServerProxy(f'http://{os.environ["user"]}:{os.environ["password"]}@localhost:{os.environ["INET_HTTP_SERVER_PORT"]}/RPC2')

configuration = {
   "groups":[
      {
         "name":"General Settings",
         "fields":[
            {
               "id":"auth-mode",
               "label":"Authentication Mode",
               "type":"text",
               "value":"JWT",
               "validation":{
                  "required":True
               },
               "description":"Authentication mechanism (JWT, OAuth, Basic)"
            },
            {
               "id":"token-expiry",
               "label":"Token Expiry (hours)",
               "type":"number",
               "value":24,
               "validation":{
                  "required":True,
                  "min":1,
                  "max":168
               },
               "description":"JWT token expiry time in hours"
            },
            {
               "id":"debug-mode",
               "label":"Debug Mode",
               "type":"checkbox",
               "value":False,
               "description":"Enable detailed logging for debugging"
            }
         ]
      },
      {
         "name":"Performance Settings",
         "fields":[
            {
               "id":"max-concurrent",
               "label":"Max Concurrent Sessions",
               "type":"number",
               "value":1000,
               "validation":{
                  "required":True,
                  "min":100,
                  "max":10000
               },
               "description":"Maximum number of concurrent user sessions"
            },
            {
               "id":"cache-ttl",
               "label":"Cache TTL",
               "type":"float",
               "value":15.5,
               "validation":{
                  "required":True,
                  "min":1,
                  "max":60
               },
               "description":"Cache time-to-live in minutes"
            }
         ]
      }
   ]
}

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


def get_status(items):
    return "active" if all(item["statename"] == "RUNNING" for item in items) else "inactive"

def is_active(items):
    return True if all(item["statename"] == "RUNNING" for item in items) else False

def get_uptime(items):
    if not items:
        return ""
    if not "description" in items[0]:
        return ""
    
    return items[0]["description"]

def get_service_config(service_id):
    print(service_id)
    try:
        service = Service.objects.get(service_id=service_id)
    except Service.DoesNotExist:
        return {"groups": []}
    
    print(service_id)
    groups = ServiceConfigGroup.objects.filter(service=service, is_active=True).order_by("order")
    groups_data = []
    
    for group in groups:
        fields_qs = ServiceConfigFieldInstance.objects.filter(group=group, is_active=True).order_by("order")
        fields_data = []
        for field in fields_qs:
            meta_info = field.definition.meta_info or {}
            fields_data.append({
                "id": field.id,
                "label": field.definition.label,
                "value": field.value,
                "default_value": field.definition.default_value,
                "type": field.definition.input_type.name if field.definition.input_type else None,
                "validation": field.definition.validation,
                "description": field.definition.description,
                "order": field.order,
                "options": field.definition.options,
                "thresholds": field.value if field.definition.input_type.name == "threshold" else None,
                **meta_info,
            })
        groups_data.append({
            "name": group.name,
            "order": group.order,
            "meta_info": group.meta_info,
            "fields": fields_data,
        })
    
    return {"groups": groups_data}


router = APIRouter(
    prefix="/api/v1",
    tags=["Service"],
    route_class=TimedRoute,
    responses={404: {"description": "Not found"}},
)

@router.api_route(
    "/services", methods=["GET"], tags=["Service"]
)
def get_service(response: Response):
    results = {}
    try:

        tenant = Tenant.objects.all().first()
        if not tenant:
            raise HTTPException(
                status_code=404, detail=f"Service not yet configured"
            )
        
        errors = get_metrics()["errors"]
        requests = get_metrics()["requests"]
        health_score = calculate_health_score(errors, requests, health_ok=True)
        grouped_data = defaultdict(list)
        items = server.supervisor.getAllProcessInfo()
        for item in items:
            status = "healthy" if all(item["statename"] == "RUNNING" for item in items) else "unhealthy"
            grouped_data[item['group']].append(item)
        
        results = {
            "id": f"rtcvision-{tenant.tenant_id}",
            "name": f"RTCVision {tenant.tenant_name}",
            "description": f"Real-Time Computer Vision @ {tenant.tenant_name} in {tenant.location}",
            "status": "active",
            "version": os.getenv("RTCVISION_VERSION", "1.0.0"),
            "environment": os.getenv("ENVIRONMENT", "production"),
            "uptime": get_supervisord_uptime(),
            "lastDeployed": get_supervisord_last_deployed(),
            "cpu": psutil.cpu_percent(interval=0.1),
            "memory": psutil.virtual_memory().percent,
            "healthScore": health_score,
            "requests": requests,
            "errors": errors,
            "tenantId": tenant.tenant_id,
            "microservices": [
                {
                    "id": group, 
                    "name": group, 
                    "status": get_status(config), 
                    "is_active": is_active(config), 
                    "version": "1.2.0",
                    "uptime": get_uptime(config),
                    "cpu": "2.3%",
                    "memory": "256MB",
                    "configuration": get_service_config(service_id=group)
                } for group, config in grouped_data.items()
            ]
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
    "/processor", methods=["GET"], tags=["Service"]
)
def get_service_data_by_service_name(response: Response, service_name:str):
    results = {}
    try:
        row = []
        grouped_data = defaultdict(list)
        row = server.supervisor.getProcessInfo(service_name)
        if not row:
            results['error'] = {
                "status_code": "not found",
                "status_description": f"query service_name {service_name} not found",
                "detail": f"query service_name {service_name} not found;",
            }
            response.status_code = status.HTTP_400_BAD_REQUEST
            
            return results
        
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
            "status_description": f"Internal Server Error",
            "detail": str(e),
        }
        
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
    
    return results

@router.api_route(
    "/group", methods=["GET"], tags=["Service"]
)
def get_service_data_by_group_name(response: Response, group_name:str):
    results = {}
    try:
        row = []
        grouped_data = defaultdict(list)
        row = server.supervisor.getAllProcessInfo()
        filterd_data = []
        for item in row:
            filterd_data.append(
                {
                    'name': item['name'],
                    'group': item['group'],
                    'start': item['start'],
                    'stop': item['stop'],
                    'is_active': item['statename'] == "RUNNING",
                    'status': 'healthy' if item['statename'] == "RUNNING" else 'unhealthy',
                    'statename': item['statename'],
                    'description': item['description'],
                }
            )
        
        
        grouped_data = [
            item for item in filterd_data if item['group'] == group_name
        ]
        
        results = {
            'data': grouped_data
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
    "/group/start", methods=["POST"], tags=["Service"]
)
def start_group(response: Response, group_name:str):
    results = {}
    try:
        row = []
        row = server.supervisor.startProcessGroup(group_name)
        
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
    "/group/stop", methods=["POST"], tags=["Service"]
)
def stop_group(response: Response, group_name:str):
    results = {}
    try:
        row = []
        row = server.supervisor.stopProcessGroup(group_name)
        
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
    
    print(results)
    print(response.status_code)
    return results

@router.api_route(
    "/processor/start", methods=["POST"], tags=["Service"]
)
def start_process(response: Response, service_name:str):
    results = {}
    try:
        row = []
        print(service_name)
        row = server.supervisor.startProcess(service_name)
        
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
            "status_description": f"Internal Server Error: {service_name}",
            "detail": str(e),
        }
        
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
    
    return results

@router.api_route(
    "/processor/stop", methods=["POST"], tags=["Service"]
)
def stop_process(response: Response, service_name:str):
    results = {}
    try:
        row = []
        row = server.supervisor.stopProcess(service_name)
        
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
            "status_description": f"Internal Server Error: {service_name}",
            "detail": str(e),
        }
        
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
    
    return results

@router.api_route(
    "/start", methods=["POST"], tags=["Service"]
)
def start_all(response: Response):
    results = {}
    try:
        row = []
        row = server.supervisor.startAllProcesses()
        
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
            "status_description": f"Internal Server Error",
            "detail": str(e),
        }
        
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
    
    return results


@router.api_route(
    "/stop", methods=["POST"], tags=["Service"]
)
def stop_all(response: Response):
    results = {}
    try:
        row = []
        row = server.supervisor.stopAllProcesses()
        
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
            "status_description": f"Internal Server Error",
            "detail": str(e),
        }
        
        response.status_code = status.HTTP_500_INTERNAL_SERVER_ERROR
    
    return results

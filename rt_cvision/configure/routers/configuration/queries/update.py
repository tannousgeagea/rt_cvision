
import os
import time
import django
django.setup()
from fastapi import Response, Request
from fastapi import FastAPI, HTTPException, Body
from pydantic import BaseModel, Field
from fastapi.routing import APIRoute
from fastapi import APIRouter
from typing import List, Any, Callable
from configure.models import (
    ServiceConfigGroup, 
    ServiceConfigFieldInstance, 
    Service,
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

class ConfigChange(BaseModel):
    # serviceId: str = Field(..., description="Unique identifier of the service")
    fieldId: int = Field(..., description="ID of the configuration field definition")
    groupName: str = Field(..., description="Name of the configuration group")
    oldValue: Any = Field(..., description="Current value that should match before updating")
    newValue: Any = Field(..., description="New value to save")

class ApiRequest(BaseModel):
    serviceId: str = Field(..., description="Unique identifier of the service")
    changes: List[ConfigChange]

class ConfigChangeResponse(BaseModel):
    serviceId: str
    fieldId: int
    groupName: str
    status: str 
    message: str

router = APIRouter(
    tags=["Configuration"],
    route_class=TimedRoute,
    responses={404: {"description": "Not found"}},
)


@router.api_route(
    "/configuration/update", tags=["Configuration"], methods=["PUT"]
)
def update_configuration(request: ApiRequest = Body(...)):
    responses: List[ConfigChangeResponse] = []

    for change in request.changes:
        try:
            service = Service.objects.get(service_id=request.serviceId)
        except Service.DoesNotExist:
            responses.append(ConfigChangeResponse(
                serviceId=request.serviceId,
                fieldId=change.fieldId,
                groupName=change.groupName,
                status="error",
                message="Service not found."
            ))
            continue

        try:
            group = ServiceConfigGroup.objects.get(service=service, name=change.groupName)
        except ServiceConfigGroup.DoesNotExist:
            responses.append(ConfigChangeResponse(
                serviceId=request.serviceId,
                fieldId=change.fieldId,
                groupName=change.groupName,
                status="error",
                message="Configuration group not found."
            ))
            continue

        try:
            field_instance = ServiceConfigFieldInstance.objects.get(group=group, pk=change.fieldId)
        except ServiceConfigFieldInstance.DoesNotExist:
            responses.append(ConfigChangeResponse(
                fieldId=change.fieldId,
                groupName=change.groupName,
                status="error",
                message="Configuration field not found in this group."
            ))
            continue

        if field_instance.value != change.oldValue:
            responses.append(ConfigChangeResponse(
                serviceId=request.serviceId,
                fieldId=change.fieldId,
                groupName=change.groupName,
                status="error",
                message=f"Old value mismatch. Current value is {field_instance.value}."
            ))
            continue

        try:
            field_instance.value = change.newValue
            field_instance.save()
            responses.append(ConfigChangeResponse(
                serviceId=request.serviceId,
                fieldId=change.fieldId,
                groupName=change.groupName,
                status="success",
                message="Field updated successfully."
            ))
        except Exception as e:
            responses.append(ConfigChangeResponse(
                serviceId=request.serviceId,
                fieldId=change.fieldId,
                groupName=change.groupName,
                status="error",
                message=str(e)
            ))

    return {
        "data": responses
    }

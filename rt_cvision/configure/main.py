import uvicorn
from uuid import uuid4
from typing import Optional, Any
from fastapi import FastAPI, Depends, APIRouter
from asgi_correlation_id import CorrelationIdMiddleware
from fastapi.middleware.cors import CORSMiddleware


from fastapi import HTTPException, Body, status, Request
from fastapi.responses import JSONResponse
from fastapi.exception_handlers import http_exception_handler
from asgi_correlation_id import correlation_id

from configure.routers.params import endpoints
from configure.routers.processor import get_processor
from configure.routers.configuration import endpoint
from configure.routers import tenant
from configure.routers import data_acquisition
from configure.routers import logs
from configure.routers import images

def create_app() -> FastAPI:
    tags_meta = [
        {
            "name": "rt-CVision DATA API",
            "description": "Real Time Computer Vision data API"
        }
    ]

    app = FastAPI(
        openapi_tags = tags_meta,
        debug=True,
        title="rt-CVision entrypoint API",
        summary="",
        version="0.0.1",
        contact={
            "name": "Tannous Geagea",
            "url": "https://wasteant.com",
            "email": "tannous.geagea@wasteant.com",            
        },
        openapi_url="/openapi.json"
    )

    origins = ["http//10.7.0.6:23085", "http://localhost:3001", "http://server2.learning.test.want:8080", "http://server2.learning.test.want:23085"]
    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_methods=["*"],
        allow_headers=["X-Requested-With", "X-Request-ID"],
        expose_headers=["X-Request-ID"],
    )

    app.include_router(endpoints.router)
    app.include_router(get_processor.router)
    app.include_router(endpoint.router)
    app.include_router(logs.endpoint.router)
    app.include_router(tenant.endpoint.router)
    app.include_router(data_acquisition.endpoint.router)
    app.include_router(images.endpoint.router)
    
    return app

app = create_app()

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "status_code": exc.status_code,
            "status_description": exc.detail,
            "detail": exc.detail
        }
    )

@app.exception_handler(Exception)
async def internal_server_error_handler(request: Request, exc: Exception):
    return JSONResponse(
        status_code=500,
        content={
            "status_code": 500,
            "status_description": "Internal Server Error",
            "detail": "An unexpected error occurred. Please try again later."
        }
    )


if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=23085, log_level="debug", reload=True)

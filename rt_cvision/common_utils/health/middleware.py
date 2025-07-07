# rtcvision/middleware.py

from starlette.middleware.base import BaseHTTPMiddleware
from starlette.requests import Request
from starlette.responses import Response
from common_utils.health.metrics import increment_error, increment_request

class MetricsMiddleware(BaseHTTPMiddleware):
    async def dispatch(self, request: Request, call_next):
        increment_request()
        try:
            response: Response = await call_next(request)
            if response.status_code >= 400:
                increment_error()
            return response
        except Exception:
            increment_error()
            raise

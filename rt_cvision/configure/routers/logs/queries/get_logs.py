import re
import os
import time
from fastapi import Response, Request
from fastapi import FastAPI, HTTPException, Body, Query
from pydantic import BaseModel, Field
from fastapi.routing import APIRoute
from fastapi import APIRouter
from typing import List, Any, Callable
from xmlrpc.client import ServerProxy

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
    tags=["Logs"],
    route_class=TimedRoute,
    responses={404: {"description": "Not found"}},
)

class LogEntry(BaseModel):
    timestamp: str
    level: str
    message: str

def parse_log(log_text: str) -> List[LogEntry]:
    """
    Parses raw log text into a list of LogEntry objects.
    Expected format per line (example):
      "[2024-02-22 10:15:30] INFO  Starting microservice..."
    Adjust the regex pattern below if your log format differs.
    """
    log_entries = []
    # Regex to capture timestamp, level, and message.
    # Assumes timestamp is inside square brackets, level is a word, and then message follows.
    pattern = re.compile(
        r"^(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d+) - (?P<level>\w+) - (?P<message>.*)$"
    )

    for line in log_text.splitlines():
        match = pattern.match(line.strip())
        if match:
            timestamp, level, message = match.groups()
            log_entries.append(LogEntry(timestamp=timestamp, level=level, message=message))
    return log_entries

@router.api_route(
    "/logs", methods=["GET"], tags=["Logs"]
)
def get_process_log(
    process_name: str,
    offset: int = Query(0, ge=0, description="Starting offset in the log file"),
    length: int = Query(1000, gt=0, description="Number of characters to read"),
    log_type: str = Query("stdout", regex="^(stdout|stderr)$", description="Log type: stdout or stderr")
):
    """
    Fetches the log for the specified process from Supervisor.
    This API calls Supervisor's XML-RPC method `tailProcessLog` or `tailProcessErrLog`
    depending on the requested log type.
    """
    try:
        server = ServerProxy(f'http://{os.environ["user"]}:{os.environ["password"]}@localhost:{os.environ["INET_HTTP_SERVER_PORT"]}/RPC2')
        if log_type == "stdout":
            log_data = server.supervisor.tailProcessStdoutLog(process_name, offset, length)
        else:
            log_data = server.supervisor.tailProcessStderrLog(process_name, offset, length)
        
        # log_data is a tuple: (log_content, new_offset, overflow)
        log_content, new_offset, overflow = log_data
        log_content = parse_log(log_content)

        return {
            "log": log_content,
            "offset": new_offset,
            "overflow": overflow
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import uvicorn
import re
import os
import time
import random
import asyncio
from fastapi import Response, Request
from fastapi import FastAPI, HTTPException, Body, Query
from pydantic import BaseModel, Field
from fastapi.routing import APIRoute
from fastapi import APIRouter
from typing import List, Any, Callable
from fastapi.responses import StreamingResponse
from xmlrpc.client import ServerProxy
app = FastAPI()


async def fake_video_streamer():
    for i in range(10):
        yield b"some fake video bytes"
router = APIRouter(
    tags=["Logs"],
    responses={404: {"description": "Not found"}},
)


@router.api_route("/test")
async def stream_logs(request: Request):
    async def event_generator():
        while True:
            if await request.is_disconnected():
                break
            # Simulate a log line using your desired format.
            # Example: "2025-03-09 08:23:02,854 - INFO - Loading base model: /path/to/model"
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            ms = f",{random.randint(100,999)}"
            level = random.choice(["INFO", "WARN", "ERROR", "DEBUG"])
            message = f"Random log message {random.randint(1, 1000)}"
            log_line = f"{timestamp}{ms} - {level} - {message}"
            print(log_line)
            yield f"data: {log_line}\n\n"
            await asyncio.sleep(3)
    return StreamingResponse(event_generator(), media_type="text/event-stream")


if __name__ == "__main__":
    uvicorn.run("test:app", host="0.0.0.0", port=23085, log_level="debug", reload=True)

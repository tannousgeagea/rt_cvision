# services/metadata.py

from datetime import datetime, timezone
from xmlrpc.client import ServerProxy
import os
import psutil
from django.utils.timezone import now
from fastapi import HTTPException
from dataclasses import dataclass

def get_supervisor_proxy():
    return ServerProxy(f"http://{os.environ['user']}:{os.environ['password']}@localhost:{os.environ['INET_HTTP_SERVER_PORT']}/RPC2")

def get_microservices():
    server = get_supervisor_proxy()
    all_info = server.supervisor.getAllProcessInfo()

    microservices = []
    for proc in all_info:
        pid = proc.get('pid')
        name = proc.get('name')
        start_time = datetime.fromtimestamp(proc.get("start")).replace(tzinfo=timezone.utc)
        print(start_time)
        print(now())
        uptime = now() - start_time

        if pid > 0 and psutil.pid_exists(pid):
            p = psutil.Process(pid)
            print(p)
            cpu = p.cpu_percent(interval=0.1)
            print("CPU:: ", cpu)
            mem = p.memory_percent()
        else:
            cpu, mem = 0, 0

        microservices.append({
            "id": name,
            "pid": pid,
            "name": name.replace("_", " ").title(),
            "status": proc.get("statename", "unknown").lower(),
            "version": None,  # optional from config
            "uptime": str(uptime).split(".")[0],
            "cpu": round(cpu),
            "memory": round(mem),
            "instances": 1,
            "port": None,
            "healthEndpoint": "/health",
            "lastRestart": start_time.isoformat(),
            "logs": [],
            "metrics": [],
            "configuration": []
        })

    return microservices

@dataclass
class Tenant:
    tenant_id:str = "wasteant"
    tenant_name:str = "WasteAnt"
    location:str = "Bremen"

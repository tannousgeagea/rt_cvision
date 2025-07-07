import psutil
from datetime import datetime, timezone
from .runtime import app_start_time

def calculate_health_score(error_count: int, request_count: int, health_ok: bool = True):
    # --- 1. CPU and Memory ---
    cpu_usage = psutil.cpu_percent(interval=0.1)
    mem_usage = psutil.virtual_memory().percent

    cpu_score = max(0, 100 - cpu_usage)     # e.g., 30% usage = 70
    mem_score = max(0, 100 - mem_usage)     # e.g., 40% usage = 60

    # --- 2. Uptime ---
    uptime = (datetime.now(tz=timezone.utc) - app_start_time).total_seconds()
    uptime_score = min(100, uptime / (24 * 3600) * 100)  # max out after 1 day

    # --- 3. Error rate ---
    error_rate = (error_count / request_count * 100) if request_count else 0
    error_score = max(0, 100 - error_rate * 2)  # penalize steeply if >5%

    # --- 4. Health check ---
    availability_score = 100 if health_ok else 0

    # --- Final Weighted Average ---
    score = (
        0.25 * cpu_score +
        0.25 * mem_score +
        0.15 * uptime_score +
        0.2  * error_score +
        0.15 * availability_score
    )

    return round(score)

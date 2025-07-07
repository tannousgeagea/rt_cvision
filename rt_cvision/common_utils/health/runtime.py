
from datetime import datetime, timezone

# This gets set once at process startup
app_start_time = datetime.now(tz=timezone.utc)

import psutil
import time
from datetime import datetime, timedelta

def get_supervisord_uptime() -> str:
    for proc in psutil.process_iter(['name', 'cmdline', 'create_time']):
        if "supervisord" in proc.info['name'].lower() or \
           any("supervisord" in arg for arg in proc.info['cmdline']):
            start_time = proc.info['create_time']
            now = time.time()
            uptime_seconds = int(now - start_time)
            return str(timedelta(seconds=uptime_seconds))
    return "Unavailable"

def get_supervisord_last_deployed() -> str:
    for proc in psutil.process_iter(['name', 'cmdline', 'create_time']):
        if "supervisord" in proc.info['name'].lower() or \
           any("supervisord" in arg for arg in proc.info['cmdline']):
            timestamp = datetime.fromtimestamp(proc.info['create_time'])
            return timestamp.isoformat()
    return "Unavailable"
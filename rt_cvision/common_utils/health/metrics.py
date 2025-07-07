# rtcvision/metrics.py

from threading import Lock

_error_count = 0
_request_count = 0
_lock = Lock()

def increment_error():
    global _error_count
    with _lock:
        _error_count += 1

def increment_request():
    global _request_count
    with _lock:
        _request_count += 1

def get_metrics():
    with _lock:
        return {
            "errors": _error_count,
            "requests": _request_count
        }

def reset_metrics():
    global _error_count, _request_count
    with _lock:
        _error_count = 0
        _request_count = 0

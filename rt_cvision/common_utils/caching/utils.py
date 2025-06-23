from typing import Dict
import hashlib

def make_cache_key(namespace: str, identifiers: Dict[str, str | int | bool]) -> str:
    """
    Generates a standardized cache key with a namespace and dictionary of key parts.

    Example:
        make_cache_key("service_config", {
            "service_id": "auth",
            "include_inactive": True,
            "format": "json"
        }) → "service_config:service_id=auth:include_inactive=1:format=json"
    """
    parts = [f"{k}={int(v) if isinstance(v, bool) else v}" for k, v in sorted(identifiers.items())]
    return f"{namespace}:" + ":".join(parts)

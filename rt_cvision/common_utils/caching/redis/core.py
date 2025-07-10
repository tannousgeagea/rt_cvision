import os
from typing import Any, Optional
from common_utils.caching.base import BaseCache
from common_utils.services.redis import redis_manager

if not redis_manager:
    raise ValueError(f"⚠️ Redis is not available.")

cache = redis_manager.redis_client

class RedisCache(BaseCache):
    def get(self, key: str) -> Optional[Any]:
        return cache.get(key)

    def set(self, key: str, value: Any, ttl: int = 60) -> None:
        cache.set(key, value)
        cache.expire(name=key, time=ttl, nx=True)

    def delete(self, key: str) -> None:
        cache.delete(key)
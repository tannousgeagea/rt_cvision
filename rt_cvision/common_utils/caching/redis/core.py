import os
from typing import Any, Optional
from common_utils.caching.base import BaseCache
from common_utils.services.redis_manager import RedisManager

redis_manager = RedisManager(
    host=os.environ['REDIS_HOST'],
    port=int(os.environ['REDIS_PORT']),
    db=int(os.environ['REDIS_DB']),
    password=os.environ['REDIS_PASSWORD'],
)

cache = redis_manager.redis_client

class RedisCache(BaseCache):
    def get(self, key: str) -> Optional[Any]:
        return cache.get(key)

    def set(self, key: str, value: Any, ttl: int = 60) -> None:
        cache.set(key, value)
        cache.expire(name=key, time=ttl, nx=True)

    def delete(self, key: str) -> None:
        cache.delete(key)
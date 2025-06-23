from .base import BaseCache
from common_utils.caching.redis.core import RedisCache

# In future, add "MemoryCache", "FileCache", etc.
def get_cache_backend(name: str = "redis") -> BaseCache:
    if name == "redis":
        return RedisCache()
    raise NotImplementedError(f"Cache backend '{name}' is not supported.")
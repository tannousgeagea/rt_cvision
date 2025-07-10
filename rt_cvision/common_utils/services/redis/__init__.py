import os
import logging
from .core import RedisManager
from redis.exceptions import RedisError, ConnectionError
from common_utils.log import Logger
logger = Logger(name="Redis Manager", level=logging.DEBUG)


def _init_redis_manager():
    try:
        host = os.getenv("REDIS_HOST", "localhost")
        port = int(os.getenv("REDIS_PORT", 6379))
        db = int(os.getenv("REDIS_DB", 0))
        password = os.getenv("REDIS_PASSWORD", None)

        manager = RedisManager(host=host, port=port, db=db, password=password)
        
        # Optional: Test connection on startup
        manager.redis_client.ping()
        logger.info(f"✅ Connected to Redis at {host}:{port}, db={db}")
        return manager

    except ValueError as e:
        logger.error(f"❌ Invalid REDIS_* environment variable: {e}")
    except ConnectionError as e:
        logger.error(f"❌ Redis connection failed: {e}")
    except RedisError as e:
        logger.error(f"❌ Redis error: {e}")
    except Exception as e:
        logger.exception(f"❌ Unexpected error initializing RedisManager: {e}")

    return None  # Fallback: return None if setup fails

redis_manager = _init_redis_manager()

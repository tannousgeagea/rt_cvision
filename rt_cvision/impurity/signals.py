import json
import os
from common_utils.services.redis import redis_manager

if not redis_manager:
    raise ValueError(f"⚠️ Redis is not available.")

r = redis_manager.redis_client 

def queue_impurity_event(instance):
    data = {
        "event": "impurity",
        "id": instance.id,
        "object_uid": instance.object_uid,
        "created_at": instance.created_at.strftime('%Y-%m-%d %H:%M:%S'),
    }
    
    print(data)
    r.rpush("impurity:queue", json.dumps(data))
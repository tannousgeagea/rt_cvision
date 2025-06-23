import django
django.setup()

import requests
import logging
import json
import time

logger = logging.getLogger(__name__)

from tenants.models import (
    PlantEntity,
    SensorBox,
)

class ServiceConfig:
    """Represents a single service configuration allowing dot-notation access."""
    def __init__(self, params):
        self.__dict__.update(params)  # Convert dict keys to attributes

    def __getattr__(self, name):
        """Return None instead of raising AttributeError if parameter is missing."""
        return None

    def __repr__(self):
        return f"<ServiceConfig {self.__dict__}>"
    
class ConfigManager:
    """Django Service Configuration Manager that allows dot notation access."""
    _instance = None  # Singleton instance

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(ConfigManager, cls).__new__(cls)
            cls._instance.reload_config()
        return cls._instance

    def reload_config(self):
        """Reload configuration from the database."""
        from configure.models import Service, ServiceParams  # Avoid early import issues

        self.services = Service.objects.all()
        self.__dict__.update({
            service.service_name: ServiceConfig({
                param.name: param.value for param in ServiceParams.objects.filter(service=service)
            })
            for service in self.services
        })

    def __repr__(self):
        return f"<ConfigManager Services={list(self.__dict__.keys())}>"

entity = PlantEntity.objects.all()
sensorbox = None
if entity:
    entity = entity.first()
    sensorbox = SensorBox.objects.get(plant_entity=entity)

class ServiceConfigClient:
    def __init__(self, api_url: str, service_id: str, timeout: int = 5, cache_ttl: int = 60):
        self.api_url = api_url.rstrip("/")
        self.service_id = service_id
        self.timeout = timeout
        self.cache_ttl = cache_ttl
        self._cache = None
        self._last_fetched = 0

    def _fetch_json(self, url: str, params: dict = None) -> dict:
        try:
            resp = requests.get(url, params=params or {}, timeout=self.timeout)
            resp.raise_for_status()
            return resp.json()
        except Exception as e:
            logger.warning(f"[ServiceConfigClient] Request failed: {e}")
            return {}

    def _fetch_config(self, include_inactive=False, format="json") -> dict:
        url = f"{self.api_url}/api/v1/services/{self.service_id}/config/flat"
        params = {
            "include_inactive": str(include_inactive).lower(),
            "format": format
        }

        try:
            r = requests.get(url, params=params, timeout=self.timeout)
            r.raise_for_status()
            if format == "json":
                return r.json()
            else:
                return self._parse_env_format(r.text)
        except Exception as e:
            logger.warning(f"[ServiceConfigClient] Failed to load config: {e}")
            return {}

    def get_tenant_context(self) -> dict:
        url = f"{self.api_url}/api/v1/tenant/config"
        return self._fetch_json(url)

    def get_data_acquisition_config(self) -> dict:
        url = f"{self.api_url}/api/v1/data-acquisition/config"
        return self._fetch_json(url)

    def get_available_sources(self, interface: str = None) -> list[dict]:
        url = f"{self.api_url}/api/v1/data-acquisition/sources"
        return self._fetch_json(url, {"interface": interface}) if interface else self._fetch_json(url)

    def _parse_env_format(self, text: str) -> dict:
        result = {}
        for line in text.strip().splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                k, v = k.strip(), v.strip()
                try:
                    result[k.lower()] = json.loads(v)
                except Exception:
                    result[k.lower()] = v
        return result

    def get(self, key: str, default=None):
        config = self.load()
        return config.get(key, default)

    def load(self, force_refresh=False) -> dict:
        now = time.time()
        if not force_refresh and self._cache and (now - self._last_fetched < self.cache_ttl):
            return self._cache

        config = self._fetch_config()
        self._cache = config
        self._last_fetched = now
        return config


if __name__ == "__main__":
    config = ServiceConfigClient(
        api_url="http://localhost:23085",
        service_id="impurity"
    )

    print(config.get("weights"))
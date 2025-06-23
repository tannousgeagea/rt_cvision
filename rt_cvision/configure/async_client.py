import httpx
import logging
import json
import time

class AsyncServiceConfigClient:
    def __init__(self, api_url: str, service_id: str, timeout: int = 5, cache_ttl: int = 60):
        self.api_url = api_url.rstrip("/")
        self.service_id = service_id
        self.timeout = timeout
        self.cache_ttl = cache_ttl
        self._cache = None
        self._last_fetched = 0

    async def _fetch_config(self, include_inactive=False, format="json") -> dict:
        url = f"{self.api_url}/api/v1/services/{self.service_id}/config/flat"
        params = {
            "include_inactive": str(include_inactive).lower(),
            "format": format
        }

        try:
            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.get(url, params=params)
                response.raise_for_status()
                if format == "json":
                    return response.json()
                else:
                    return self._parse_env_format(response.text)
        except Exception as e:
            logger.warning(f"[AsyncServiceConfigClient] Failed to load config: {e}")
            return {}

    async def _fetch_json(self, url: str, params: dict = None) -> dict:
        try:
            async with httpx.AsyncClient(timeout=self.timeout) as client:
                resp = await client.get(url, params=params or {})
                resp.raise_for_status()
                return resp.json()
        except Exception as e:
            logger.warning(f"[AsyncServiceConfigClient] Request failed: {e}")
            return {}
        
    async def get_tenant_context(self) -> dict:
        url = f"{self.api_url}/api/v1/tenant/config"
        return await self._fetch_json(url)

    async def get_data_acquisition_config(self) -> dict:
        url = f"{self.api_url}/api/v1/data-acquisition/config"
        return await self._fetch_json(url)

    async def get_available_sources(self, interface: str = None) -> list[dict]:
        url = f"{self.api_url}/api/v1/data-acquisition/sources"
        return await self._fetch_json(url, {"interface": interface}) if interface else await self._fetch_json(url)

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

    async def get(self, key: str, default=None):
        config = await self.load()
        return config.get(key, default)

    async def load(self, force_refresh=False) -> dict:
        now = time.time()
        if not force_refresh and self._cache and (now - self._last_fetched < self.cache_ttl):
            return self._cache

        config = await self._fetch_config()
        self._cache = config
        self._last_fetched = now
        return config


async def main():
    config = AsyncServiceConfigClient(
        api_url="http://localhost:23085",
        service_id="impurity",
    )

    tenant = await config.get_tenant_context()
    data_source = await config.get_data_acquisition_config()
    sources = await config.get_available_sources("ros2")

    print(tenant)
    print(data_source)
    print(sources)

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())
import httpx
import requests

def get_param(service_id: str, param: str) -> str:
    url = f"http://localhost:23085/api/v1/params/{service_id}/{param}"
    response = requests.get(url)

    if response.status_code == 200:
        data = response.json()
        return data["value"]
    else:
        raise ValueError(f"Failed to fetch param. Status code: {response.status_code}, Response: {response.text}")

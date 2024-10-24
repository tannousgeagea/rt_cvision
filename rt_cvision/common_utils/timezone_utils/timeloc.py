import requests
import logging
from datetime import datetime, timezone
import pytz

def get_location_and_timezone():
    timezone = "Europe/Berlin"
    try:
        response = requests.get("https://ipinfo.io")
        data = response.json()
        timezone = data['timezone']
    except Exception as err:
        logging.error(f'Failed to get local timezone: {err}')

    return timezone

def convert_to_local_time(utc_time:datetime, timezone_str:str):
    if utc_time.tzinfo is None:
        utc_time = pytz.utc.localize(utc_time)
        
    local_timezone = pytz.timezone(timezone_str)
    local_time = utc_time.astimezone(local_timezone)
    return local_time

# # Get the user's timezone
# timezone_str = get_location_and_timezone()
# print(timezone_str)

# utc_time = datetime.now(tz=timezone.utc)
# local_time = convert_to_local_time(timezone_str=timezone_str, utc_time=utc_time)
# print(f"Local time in {timezone_str}: {local_time}")
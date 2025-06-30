
from typing import List, Dict, Union

def map_attributes(attrs:List[str]) -> Dict[str, Union[str, bool]]:
    attrs_dict = {}
    if not attrs:
        return attrs_dict
    
    for attr in attrs:
        if ":" in attr:
            key, value = attr.split(":")
            attrs_dict[key] = value
        else:
            attrs_dict[attr] = True
    
    return attrs_dict

IMPURITY_RULES = [
    {
        "label": "Hard & Oversized",
        "conditions": [
            ["rigid", "long"],
            ["predicted_severity:high", "long"]
        ],
        "severity": "high",
        "class_id": 3,
        "weight_threshold": 2,
        "color": (0, 0, 204),  # 🔴 dark red
    },
    {
        "label": "Prohibited Material",
        "conditions": [
            ["manmade", "predicted_severity:high"],
            ["rigid", "predicted_severity:high"],
        ],
        "severity": "high",
        "class_id": 3,
        "weight_threshold": 1.2,
        "color": (0, 0, 255),  # 🔴 red
        
    },
    {
        "label": "Potentially Dangerous Shape",
        "conditions": [
            ["fractured", "predicted_severity:high"],
            ["predicted_severity:high"],
            ["rigid"],
        ],
        "severity": "medium",
        "class_id": 2,
        "color": (0, 85, 235),  # 🟧 orange
    },
    {
        "label": "Model Detected Rigid",
        "conditions": [
            ["rigid", "fractured"]
        ],
        "severity": "medium",
        "class_id": 2,
        "weight_threshold": 0.9,
        "color": (0, 85, 200),  # 🟧 orange
    },
    {
        "label": "Medium Risk",
        "conditions": [
            ["rigid", "fractured"], 
            ["predicted_severity:medium", "manmade"],
            ["predicted_severity:medium"]
        ],
        "severity": "medium",
        "class_id": 2,
        "weight_threshold": 0.7,
        "color": (0, 204, 255),  # 🟨 gold
    },
    {
        "label": "Low Risk",
        "conditions":[
            ["predicted_severity:low"],
            ["predicted_severity:low", "manmade"],
            ["predicted_severity:low", "fractured"]
        ],
        "severity": "low",
        "class_id": 1,
        "weight_threshold": 0.5,
        "color": (204, 0, 102),  # 🟪 purple
    }
]
from typing import Dict, List, Union, Optional


class ImpurityClassifier:
    def __init__(self, rules: List[Dict]):
        self.rules = rules

    def _check_condition(self, attributes: Dict, condition: str) -> bool:
        if ":" in condition:
            key, value = condition.split(":")
            return attributes.get(key) == value
        return attributes.get(condition, False)

    def classify(
        self,
        attributes: Dict[str, Union[bool, str]],
        contour: Optional[List[List[int]]] = None
    ) -> Dict:
        for rule in self.rules:
            for condition_set in rule["conditions"]:
                if all(self._check_condition(attributes, cond) for cond in condition_set):
                    return {
                        "impurity_type": rule["label"],
                        "severity": rule["severity"],
                        "class_id": rule["class_id"],
                        "reason": condition_set,
                        "contour": contour,
                        "class_label": attributes.get("class_label"),
                        "color": rule["color"],
                    }

        return {
            "impurity_type": None,
            "severity": "low",
            "class_id": 0,
            "reason": [],
            "contour": contour,
            "class_label": attributes.get("class_label"),
        }

from typing import Dict, List, Union, Optional, Any
from common_utils.policy.utils import ClassificationRule, DowngradeEngine, ClassificationResult, Severity


# class ImpurityClassifier:
#     def __init__(self, rules: List[Dict]):
#         self.rules = rules

#     def _check_condition(self, attributes: Dict, condition: str) -> bool:
#         if ":" in condition:
#             key, value = condition.split(":")
#             return attributes.get(key) == value
#         return attributes.get(condition, False)

#     def classify(
#         self,
#         attributes: Dict[str, Union[bool, str]],
#         contour: Optional[List[List[int]]] = None
#     ) -> Dict:
#         for rule in self.rules:
#             for condition_set in rule["conditions"]:
#                 if all(self._check_condition(attributes, cond) for cond in condition_set):
#                     return {
#                         "impurity_type": rule["label"],
#                         "severity": rule["severity"],
#                         "class_id": rule["class_id"],
#                         "reason": condition_set,
#                         "contour": contour,
#                         "class_label": attributes.get("class_label"),
#                         "color": rule["color"],
#                     }

#         return {
#             "impurity_type": None,
#             "severity": "low",
#             "class_id": 0,
#             "reason": [],
#             "contour": contour,
#             "class_label": attributes.get("class_label"),
#         }



class ImpurityClassifier:
    """Main classifier with modular downgrade support"""
    
    def __init__(self, rules: List[Dict[str, Any]], downgrade_rules: List[Dict[str, Any]] = None):
        self.classification_rules = [ClassificationRule(rule) for rule in rules]
        self.downgrade_engine = DowngradeEngine(downgrade_rules or [])
        
        # Sort classification rules by priority
        self.classification_rules.sort(key=lambda x: x.priority, reverse=True)
    
    def classify(self, attributes: Dict[str, Any], 
                contour: Optional[List[List[int]]] = None) -> ClassificationResult:
        """
        Classify an object based on its attributes
        
        Args:
            attributes: Dictionary of object attributes
            contour: Optional contour information
            
        Returns:
            ClassificationResult: Classification result (potentially downgraded)
        """
        # Validate inputs
        if not isinstance(attributes, dict):
            raise ValueError("Attributes must be a dictionary")
        
        # Try each classification rule
        for rule in self.classification_rules:
            matching_conditions = rule.matches(attributes)
            if matching_conditions:
                result = rule.create_result(matching_conditions, attributes, contour)
                # Apply downgrade rules
                return self.downgrade_engine.apply_downgrades(result, attributes)
        
        # Default case
        default_result = ClassificationResult(
            impurity_type=None,
            severity=Severity.LOW,
            class_id=0,
            reason=[],
            contour=contour,
            class_label=attributes.get("class_label"),
            metadata={"default_classification": True}
        )
        
        return self.downgrade_engine.apply_downgrades(default_result, attributes)
    
    def add_downgrade_rule(self, rule_config: Dict[str, Any]):
        """Add a new downgrade rule"""
        self.downgrade_engine.add_rule(rule_config)
    
    def remove_downgrade_rule(self, trigger_attribute: str):
        """Remove a downgrade rule"""
        self.downgrade_engine.remove_rule(trigger_attribute)
    
    def get_stats(self) -> Dict[str, Any]:
        """Get classifier statistics"""
        return {
            "classification_rules": len(self.classification_rules),
            "downgrade_rules": len(self.downgrade_engine.rules),
            "available_severities": [s.value for s in Severity],
        }

from typing import List, Dict, Union
from typing import Dict, List, Union, Optional, Any
from abc import ABC, abstractmethod
from dataclasses import dataclass
from dataclasses import asdict
from enum import Enum
import logging

class Severity(Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"

@dataclass
class ClassificationResult:
    impurity_type: Optional[str]
    severity: Severity
    class_id: int
    reason: List[str]
    contour: Optional[List[List[int]]] = None
    class_label: Optional[str] = None
    color: Optional[tuple] = None
    metadata: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        result = asdict(self)
        result['severity'] = self.severity.value
        return result
        
    def to_json_dict(self) -> Dict[str, Any]:
        result = asdict(self)
        result['severity'] = self.severity.value
        if result['color'] is not None:
            result['color'] = list(result['color'])  # Convert tuple to list
        return result


class ConditionEvaluator:
    """Handles condition evaluation logic"""
    
    @staticmethod
    def evaluate_condition(attributes: Dict[str, Any], condition: str) -> bool:
        """
        Evaluate a single condition against attributes
        
        Args:
            attributes: Dictionary of object attributes
            condition: Condition string (e.g., "rigid" or "predicted_severity:high")
            
        Returns:
            bool: True if condition is met
        """
        try:
            if ":" in condition:
                key, expected_value = condition.split(":", 1)
                actual_value = attributes.get(key)
                return actual_value == expected_value
            return bool(attributes.get(condition, False))
        except Exception as e:
            logging.warning(f"Error evaluating condition '{condition}': {e}")
            return False
    
    @staticmethod
    def evaluate_condition_set(attributes: Dict[str, Any], condition_set: List[str]) -> bool:
        """
        Evaluate a set of conditions (all must be true)
        
        Args:
            attributes: Dictionary of object attributes
            condition_set: List of condition strings
            
        Returns:
            bool: True if all conditions are met
        """
        return all(ConditionEvaluator.evaluate_condition(attributes, cond) 
                  for cond in condition_set)


class DowngradeRule:
    """Represents a single downgrade rule"""
    
    def __init__(self, config: Dict[str, Any]):
        self.trigger_attribute = config["trigger_attribute"]
        self.target_severity = Severity(config["target_severity"])
        self.target_class_id = config["target_class_id"]
        self.label = config.get("label")
        self.color = config.get("color")
        self.metadata = config.get("metadata", {})
        
        # Flexible condition matching
        self.with_additional_attributes = config.get("with_additional_attributes", [])
        self.require_any_additional = config.get("require_any_additional", True)
        self.exclude_attributes = config.get("exclude_attributes", [])
        self.min_additional_count = config.get("min_additional_count", 1)
        
        # Advanced conditions
        self.custom_conditions = config.get("custom_conditions", [])
        self.priority = config.get("priority", 0)
    
    def applies_to(self, attributes: Dict[str, Any]) -> bool:
        """
        Check if this downgrade rule applies to the given attributes
        
        Args:
            attributes: Dictionary of object attributes
            
        Returns:
            bool: True if rule should be applied
        """
        # Must have trigger attribute
        if not attributes.get(self.trigger_attribute, False):
            return False
        
        # Check excluded attributes
        if any(attributes.get(attr, False) for attr in self.exclude_attributes):
            return False
        
        # Check custom conditions
        if self.custom_conditions:
            for condition_set in self.custom_conditions:
                if not ConditionEvaluator.evaluate_condition_set(attributes, condition_set):
                    return False
        
        # Check additional attributes requirements
        if self.with_additional_attributes:
            # Specific additional attributes required
            matching_attrs = sum(1 for attr in self.with_additional_attributes 
                               if attributes.get(attr, False))
            return matching_attrs >= self.min_additional_count
        
        elif self.require_any_additional:
            # Any additional attribute required (excluding trigger)
            other_attrs = sum(1 for key, value in attributes.items() 
                            if value and key != self.trigger_attribute)
            return other_attrs >= self.min_additional_count
        
        return True
    
    def apply(self, original_result: ClassificationResult, attributes: Dict[str, Any]) -> ClassificationResult:
        """
        Apply the downgrade rule to create a new result
        
        Args:
            original_result: Original classification result
            attributes: Object attributes
            
        Returns:
            ClassificationResult: New downgraded result
        """
        label = self.label or f"{original_result.impurity_type} (Downgraded)"
        
        return ClassificationResult(
            impurity_type=label,
            severity=self.target_severity,
            class_id=self.target_class_id,
            reason=[self.trigger_attribute, "downgraded"] + original_result.reason,
            contour=original_result.contour,
            class_label=original_result.class_label,
            color=self.color or original_result.color,
            metadata={
                **(original_result.metadata or {}),
                **self.metadata,
                "downgrade_applied": True,
                "original_severity": original_result.severity.value,
                "original_class_id": original_result.class_id,
            }
        )

class DowngradeEngine:
    """Manages and applies downgrade rules"""
    
    def __init__(self, downgrade_rules: List[Dict[str, Any]]):
        self.rules = [DowngradeRule(rule_config) for rule_config in downgrade_rules]
        # Sort by priority (higher first)
        self.rules.sort(key=lambda x: x.priority, reverse=True)
    
    def apply_downgrades(self, result: ClassificationResult, attributes: Dict[str, Any]) -> ClassificationResult:
        """
        Apply applicable downgrade rules to the result
        
        Args:
            result: Original classification result
            attributes: Object attributes
            
        Returns:
            ClassificationResult: Potentially downgraded result
        """
        current_result = result
        
        for rule in self.rules:
            if rule.applies_to(attributes):
                current_result = rule.apply(current_result, attributes)
                logging.info(f"Applied downgrade rule: {rule.trigger_attribute} -> {rule.target_severity.value}")
                break  # Apply only first matching rule
        
        return current_result
    
    def add_rule(self, rule_config: Dict[str, Any]):
        """Add a new downgrade rule"""
        new_rule = DowngradeRule(rule_config)
        self.rules.append(new_rule)
        self.rules.sort(key=lambda x: x.priority, reverse=True)
    
    def remove_rule(self, trigger_attribute: str):
        """Remove downgrade rule by trigger attribute"""
        self.rules = [rule for rule in self.rules if rule.trigger_attribute != trigger_attribute]

class ClassificationRule:
    """Represents a single classification rule"""
    
    def __init__(self, config: Dict[str, Any]):
        self.label = config["label"]
        self.conditions = config["conditions"]
        self.severity = Severity(config["severity"])
        self.class_id = config["class_id"]
        self.color = config.get("color")
        self.weight_threshold = config.get("weight_threshold")
        self.metadata = config.get("metadata", {})
        self.priority = config.get("priority", 0)
    
    def matches(self, attributes: Dict[str, Any]) -> Optional[List[str]]:
        """
        Check if attributes match this rule
        
        Args:
            attributes: Object attributes
            
        Returns:
            Optional[List[str]]: Matching condition set or None
        """
        for condition_set in self.conditions:
            if ConditionEvaluator.evaluate_condition_set(attributes, condition_set):
                return condition_set
        return None
    
    def create_result(self, condition_set: List[str], attributes: Dict[str, Any], 
                     contour: Optional[List[List[int]]] = None) -> ClassificationResult:
        """Create classification result from this rule"""
        return ClassificationResult(
            impurity_type=self.label,
            severity=self.severity,
            class_id=self.class_id,
            reason=condition_set,
            contour=contour,
            class_label=self.label,
            color=self.color,
            metadata={
                **self.metadata,
                "weight_threshold": self.weight_threshold,
                "rule_priority": self.priority,
            }
        )



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
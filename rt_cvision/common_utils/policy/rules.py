IMPURITY_RULES = [
    {
        "label": "Hard & Oversized",
        "conditions": [
            ["rigid", "long"],
            ["predicted_severity:high", "long"],
            ["pipe", "long"],
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
            ["pipe"],
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
            ["mattress"],
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

# Flexible downgrade rules
DOWNGRADE_RULES = [
    {
        "trigger_attribute": "mattress",
        "target_severity": "low",
        "target_class_id": 1,
        "label": "Low Risk (Mattress Override)",
        "color": (204, 0, 102),
        "require_any_additional": True,
        "min_additional_count": 1,
        "priority": 100,
    },
    {
        "trigger_attribute": "furniture",
        "target_severity": "low",
        "target_class_id": 1,
        "label": "Low Risk (Furniture Override)",
        "color": (204, 0, 102),
        "with_additional_attributes": ["rigid", "fractured"],
        "min_additional_count": 1,
        "priority": 90,
    },
    {
        "trigger_attribute": "electronic_device",
        "target_severity": "medium",
        "target_class_id": 2,
        "label": "Medium Risk (Electronic Override)",
        "color": (0, 204, 255),
        "exclude_attributes": ["battery"],  # Don't downgrade if battery present
        "priority": 80,
    },
    {
        "trigger_attribute": "textile",
        "target_severity": "low",
        "target_class_id": 1,
        "custom_conditions": [
            ["large"],  # Only if large
        ],
        "priority": 70,
    }
]


from common_utils.policy.core import ImpurityClassifier

classifier = ImpurityClassifier(IMPURITY_RULES, DOWNGRADE_RULES)

# Test cases
test_cases = [
    {"mattress": True},  # Normal classification
    {"mattress": True, "rigid": True},  # Should be downgraded
    {"furniture": True, "rigid": True, "predicted_severity": "high"},  # Should be downgraded
    {"electronic_device": True, "predicted_severity": "high"},  # Should be downgraded
    {"electronic_device": True, "battery": True, "predicted_severity": "high"},  # Should NOT be downgraded
    {"textile": True, "large": True, "predicted_severity": "high"},  # Should be downgraded
    {"rigid": True, "predicted_severity": "high"},  # Normal classification, no downgrade
]

print("=== Classification Results ===")
for i, attrs in enumerate(test_cases, 1):
    result = classifier.classify(attrs)
    print(f"\nTest {i}: {attrs}")
    print(f"  Result: {result.impurity_type}")
    print(f"  Severity: {result.severity.value}")
    print(f"  Class ID: {result.class_id}")
    print(f"  Reason: {result.reason}")
    if result.metadata and result.metadata.get("downgrade_applied"):
        print(f"  Original Severity: {result.metadata['original_severity']}")

print(f"\n=== Classifier Stats ===")
print(classifier.get_stats())
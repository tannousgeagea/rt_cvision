

def SEVERITY_LEVEL_MAP_BY_SIZE(o, thresholds:list=[0, 1., 1.5, 2.]):
    o = round(o * 100)
    for i, threshold in enumerate(thresholds):
        if o < threshold * 100:
            return (i - 1)
        
    return len(thresholds) - 1

def SEVERITY_LEVEL_MAP_BY_CLASS(c, thresholds:list=[0, 1, 2, 3]):
    if c > len(thresholds):
        return len(thresholds) - 1
    
    return thresholds[c]

SEVERITY_MAP = {
    "object_length": SEVERITY_LEVEL_MAP_BY_SIZE,
    "class_id": SEVERITY_LEVEL_MAP_BY_SIZE,
}
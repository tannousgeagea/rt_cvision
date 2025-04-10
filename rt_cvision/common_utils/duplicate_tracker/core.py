import time

class DuplicateTracker:
    def __init__(self, buffer_size=10, iou_threshold=0.5, expiry_minutes=30):
        self.buffer_size = buffer_size
        self.iou_threshold = iou_threshold
        self.expiry_seconds = expiry_minutes * 60
        self.detections = []

    def add_detection(self, detection):
        """
        Process a new detection.
        detection: dict with keys "bbox" and "timestamp"
        Returns True if the detection is new (i.e., not a duplicate).
        """
        self._cleanup(detection['timestamp'])
        
        if self.is_duplicate(detection):
            return False
        
        if len(self.detections) >= self.buffer_size:
            self.detections.pop(0)
        self.detections.append(detection)
        return True

    def is_duplicate(self, new_det):
        """
        Compare the new detection against each detection in the buffer using IoU.
        """
        for det in self.detections:
            if self.compute_iou(new_det['bbox'], det['bbox']) > self.iou_threshold:
                return True
        return False

    def _cleanup(self, current_time):
        """
        Remove detections that have expired (older than expiry_seconds relative to current_time).
        """
        self.detections = [
            det for det in self.detections
            if current_time - det['timestamp'] <= self.expiry_seconds
        ]

    def compute_iou(self, bbox1, bbox2, epsilon=1e-5):
        """
        Compute the Intersection-over-Union (IoU) of two bounding boxes.
        bbox format: [x_min, y_min, x_max, y_max]
        """
        x1, y1, x2, y2 = bbox1
        x1_p, y1_p, x2_p, y2_p = bbox2
        
        inter_x1 = max(x1, x1_p)
        inter_y1 = max(y1, y1_p)
        inter_x2 = min(x2, x2_p)
        inter_y2 = min(y2, y2_p)
        
        if inter_x2 < inter_x1 or inter_y2 < inter_y1:
            return 0.0
        
        inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)

        area1 = (x2 - x1) * (y2 - y1)
        area2 = (x2_p - x1_p) * (y2_p - y1_p)
        union_area = area1 + area2 - inter_area
        
        return inter_area / (union_area + epsilon)
    
if __name__ == "__main__":

    tracker = DuplicateTracker(buffer_size=10, iou_threshold=0.5, expiry_minutes=30)
    
    Detection1 = {
        'xyxy': [
            [2230.28955078125, 614.9255981445312, 2362.9853515625, 966.36376953125]
            ], 
        'xyxyn': [
            [0.9110659956932068, 0.3002566397190094, 0.9652717709541321, 0.47185730934143066]
        ], 
        'confidence_score': [0.8085788488388062], 
        'class_id': [1], 
        'tracker_id': [220853], 
        'object_length': [1.5814717712402342], 
        'object_area': [0.009301747332177257], 
        'object_uid': ['35210324-e4fc-4326-83b7-629f91e97b8c'], 
        'severity_level': [2]
        }

    Detection2 = {
        'xyxy': [
            [2268.97265625, 619.819091796875, 2446.14208984375, 981.15380859375]
            ], 
        'xyxyn': [
            [0.9268679022789001, 0.30264604091644287, 0.999241054058075, 0.479079008102417]
            ], 
        'confidence_score': [0.9195353388786316], 
        'class_id': [1], 
        'tracker_id': [220853], 
        'object_length': [1.6260062255859373], 
        'object_area': [0.012769009913000673], 
        'object_uid': ['35210324-e4fc-4326-83b7-629f91e97b8c'], 
        'severity_level': [2]
        }
    
    for i, xyxy in enumerate(Detection1['xyxyn']):
        temp_det = {"bbox": xyxy, "timestamp": time.time()}
        if tracker.add_detection(temp_det):
            print(f"Detection {i} is unique and added to the buffer.")
        else:
            print(f"Detection {i} is a duplicate.")

    unique_indices = []
    for i, xyxy in enumerate(Detection2['xyxyn']):
        temp_det = {"bbox": xyxy, "timestamp": time.time()}
        if tracker.add_detection(temp_det):
            print(f"Detection {i} is unique and added to the buffer.")
            unique_indices.append(i)
        else:
            print(f"Detection {i} is a duplicate.")

    problematic_objects = {
        key: [value[i] for i in unique_indices]
        for key, value in Detection2.items()
        if isinstance(value, list)
    }

    print(problematic_objects)
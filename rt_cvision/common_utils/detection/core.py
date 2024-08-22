
import os
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Iterator, Dict, Tuple, Union, List
from .utils import box_non_max_suppression, get_data_item, get_data_from_list
from .convertor import xyxy2xywh

@dataclass
class Detections:
    """
    Data class containing information about the detections.
    Attributes:
        xyxy (List): An array of shape `(n, 4)` containing
            the bounding boxes coordinates in format `[x1, y1, x2, y2]`
        poly: (List[Tuple[int, int]]): A list of tuples, where each tuple represents a vertex (x, y) of the polygon.
            `(n, H, W)` containing the segmentation masks.
        confidence (Optional[np.ndarray]): An array of shape
            `(n,)` containing the confidence scores of the detections.
        class_id (Optional[np.ndarray]): An array of shape
            `(n,)` containing the class ids of the detections.
        tracker_id (Optional[np.ndarray]): An array of shape
            `(n,)` containing the tracker ids of the detections.
        object_length (Optional[np.ndarray]): An array of shape
            `(n,)` containing the object_length of the detections.
        object_area (Optional[np.ndarray]): An array of shape
            `(n,)` containing the object area of the detections.
    """
        
    xyxy: np.ndarray
    xyxyn: np.ndarray
    xy: Optional[np.ndarray] = None
    xyn: Optional[np.ndarray] = None
    confidence: Optional[np.ndarray] = None
    class_id: Optional[np.ndarray] = None
    tracker_id: Optional[np.ndarray] = None
    object_length: Optional[np.ndarray] = None
    object_area: Optional[np.ndarray] = None
    data: Dict[str, Union[np.ndarray, List]] = field(default_factory=dict)
    
    
    def __len__(self):
        """
        Returns the number of detections in the Detections object.
        """
        return len(self.xyxy)
    
    def __iter__(
        self,
    ) -> Iterator[
        Tuple[
            np.ndarray,
            np.ndarray,
            Optional[np.ndarray],
            Optional[np.ndarray],
            Optional[float],
            Optional[int],
            Optional[int],
            Optional[float],
            Optional[float],
            Dict[str, Union[np.ndarray, List]],
        ]
    ]:
        """
        Iterates over the Detections object and yield a tuple of
        `(xyxy, xyxyn, xy, xyxyn confidence, class_id, tracker_id, object_length, object_area)` for each detection.
        """
        for i in range(len(self.xyxy)):
            yield (
                self.xyxy[i],
                self.xyxyn[i],
                get_data_from_list(self.xy, i) if self.xy is not None else None,
                get_data_from_list(self.xyn, i) if self.xyn is not None else None,
                self.confidence[i] if self.confidence is not None else None,
                self.class_id[i] if self.class_id is not None else None,
                self.tracker_id[i] if self.tracker_id is not None else None,
                self.object_length[i] if self.object_length is not None else None,
                self.object_area[i] if self.object_area is not None else None,
                get_data_item(self.data, i)
            )


    def __eq__(self, other: 'Detections'):
        return all(
            [
                np.array_equal(self.xyxy, other.xyxy),
                np.array_equal(self.class_id, other.class_id),
                np.array_equal(self.confidence, other.confidence),
                np.array_equal(self.tracker_id, other.tracker_id),
            ]
        )
        
    def __getitem__(
        self, index: Union[int, slice, List[int], np.ndarray, str]
    ) -> Union['Detections', List, np.ndarray, None]:
        """
        Get a subset of the Detections object or access an item from its data field.

        When provided with an integer, slice, list of integers, or a numpy array, this
        method returns a new Detections object that represents a subset of the original
        detections. When provided with a string, it accesses the corresponding item in
        the data dictionary.

        Args:
            index (Union[int, slice, List[int], np.ndarray, str]): The index, indices,
                or key to access a subset of the Detections or an item from the data.

        Returns:
            Union[Detections, Any]: A subset of the Detections object or an item from
                the data field.

        Example:
            ```python

            detections = Detections()

            first_detection = detections[0]
            first_10_detections = detections[0:10]
            some_detections = detections[[0, 2, 4]]
            class_0_detections = detections[detections.class_id == 0]
            high_confidence_detections = detections[detections.confidence > 0.5]

            feature_vector = detections['feature_vector']
            ```
        """
        if isinstance(index, int):
            index = [index]
        return Detections(
            xyxy=self.xyxy[index],
            xyxyn=self.xyxyn[index],
            xy=get_data_from_list(self.xy, index) if self.xy is not None else None,
            xyn=get_data_from_list(self.xyn, index) if self.xyn is not None else None,
            confidence=self.confidence[index] if self.confidence is not None else None,
            class_id=self.class_id[index] if self.class_id is not None else None,
            tracker_id=self.tracker_id[index] if self.tracker_id is not None else None,
            object_length=self.object_length[index] if self.object_length is not None else None,
            object_area=self.object_area[index] if self.object_area is not None else None,
            data=get_data_item(self.data, index),
        )
        
    @property
    def box_area(self) -> np.ndarray:
        """
        Calculate the area of each bounding box in the set of object detections.

        Returns:
            np.ndarray: An array of floats containing the area of each bounding
                box in the format of `(area_1, area_2, , area_n)`,
                where n is the number of detections.
        """
        return (self.xyxy[:, 3] - self.xyxy[:, 1]) * (self.xyxy[:, 2] - self.xyxy[:, 0])

    def with_nms(
        self, threshold: float = 0.5, class_agnostic: bool = False
    ) -> 'Detections':
        """
        Performs non-max suppression on detection set. If the detections result
        from a segmentation model, the IoU mask is applied. Otherwise, box IoU is used.

        Args:
            threshold (float, optional): The intersection-over-union threshold
                to use for non-maximum suppression. I'm the lower the value the more
                restrictive the NMS becomes. Defaults to 0.5.
            class_agnostic (bool, optional): Whether to perform class-agnostic
                non-maximum suppression. If True, the class_id of each detection
                will be ignored. Defaults to False.

        Returns:
            Detections: A new Detections object containing the subset of detections
                after non-maximum suppression.

        Raises:
            AssertionError: If `confidence` is None and class_agnostic is False.
                If `class_id` is None and class_agnostic is False.
        """
        if len(self) == 0:
            return self

        assert (
            self.confidence is not None
        ), "Detections confidence must be given for NMS to be executed."

        if class_agnostic:
            predictions = np.hstack((self.xyxy, self.confidence.reshape(-1, 1)))
        else:
            assert self.class_id is not None, (
                "Detections class_id must be given for NMS to be executed. If you"
                " intended to perform class agnostic NMS set class_agnostic=True."
            )
            predictions = np.hstack(
                (
                    self.xyxy,
                    self.confidence.reshape(-1, 1),
                    self.class_id.reshape(-1, 1),
                )
            )

        indices = box_non_max_suppression(
            predictions=predictions, iou_threshold=threshold
        )

        return self[indices]

    def to_dict(self):
        return {
            'xyxy': self.xyxy.tolist(),
            'xyxyn': self.xyxyn.tolist(),
            'xy': [xy.tolist() for xy in self.xy] if self.xy is not None else self.xy,
            'xyn': [xyn.tolist() for xyn in self.xyn] if self.xyn is not None else self.xyn,
            'confidence_score': self.confidence.tolist() if self.confidence is not None else self.confidence,
            'class_id': self.class_id.tolist() if self.class_id is not None else self.class_id,
            'tracker_id': self.tracker_id.tolist() if self.tracker_id is not None else self.tracker_id,
            'object_length': self.object_length.tolist() if self.object_length is not None else self.object_length,
            'object_area': self.object_area.tolist() if self.object_area is not None else self.object_area,
        }
        
    @classmethod
    def from_dict(cls, results) -> 'Detections':
        assert isinstance(results, dict), f'results are expected to be of type dict, but got {type(results)}'
        return Detections(
            xyxy=np.array(results.get('xyxy', [])),
            xyxyn=np.array(results.get('xyxyn', [])),
            xy=results.get('xy', None) if 'xy' in results.keys() else None,
            xyn=results.get('xyn', None) if 'xyn' in results.keys() else None,
            confidence=np.array(results.get('confidence_score', None)) if 'confidence_score' in results.keys() else None,
            class_id=np.array(results.get('class_id', None)) if 'class_id' in results.keys() else None,
            tracker_id=np.array(results.get('tracker_id', None)) if 'tracker_id' in results.keys() else None,
            object_length=np.array(results.get('object_length', None)) if 'object_length' in results.keys() else None,
            object_area=np.array(results.get('object_area', None)) if 'object_area' in results.keys() else None,
        )
    
    def to_txt(self, txt_file:str, task:str='detect',):
        
        lines = []
        if len(self.xyxy):
            assert self.class_id is not None, f'class_id must be given and not None'
            
            if task == 'detect':
                classes = np.expand_dims(self.class_id, axis=1)
                coords = np.array([xyxy2xywh(xyxy) for xyxy in self.xyxyn])
                if len(coords):
                    lines = [("%g " * len(line)).rstrip() %tuple(line) + "\n" for line in np.hstack((classes.astype(int), coords))]
                    
            if task == 'segment':
                assert self.xyn is not None, f'xyn must be given and not None'
                for i, xyn in enumerate(self.xyn):
                    xyn = xyn.flatten().tolist()
                    line = (self.class_id[i], *xyn)
                    lines.append(("%g " * len(line)).rstrip() %line + "\n")
                
        os.makedirs(os.path.dirname(txt_file), exist_ok=True)
        with open(txt_file, "w") as f:
            f.writelines(lines)


if __name__ == "__main__":
    
    from ultralytics import YOLO
    import cv2
    from glob import glob
    from common_utils.annotate.core import Annotator
    from convertor import extract_xyxy_from_txtfile
    from convertor import xywh2xyxy, xyxyn2xyxy
    
    models = YOLO('/home/appuser/data/models/yolov8-seg.gml_luh_v02.pt')
    images = glob('/home/appuser/data/images/*.jpg')
    for image in images:
        # image = '/home/appuser/data/images/StoerstofDetection_1722417190295263995_mp4-0006.jpg'
        cv_image = cv2.imread(image)
        result = models.predict(image, conf=0.05)
        
        boxes = result[0].boxes
        masks = result[0].masks
        
        detections = Detections(
            xyxy=boxes.xyxy.cpu().numpy(),
            xyxyn=boxes.xyxyn.cpu().numpy(),
            xy=masks.xy,
            xyn=masks.xyn,
            class_id=boxes.cls.cpu().numpy(),
            confidence=boxes.conf.cpu().numpy(),  
        )
        
        
        
        filename = os.path.basename(image).split('.jpg')[0] + '.txt'
        detections.to_txt(txt_file=f'/home/appuser/data/labels/{filename}', task='segment')
        print(len(detections))
        
        # class_id, boxes = extract_xyxy_from_txtfile(txt_file=f'/home/appuser/data/labels/{filename}')
        # annotator = Annotator(im=cv_image.copy(), line_width=2)
        # for xyxyn in boxes:
        #     xyxy = xyxyn2xyxy(xyxyn=xyxyn, image_shape=cv_image.shape)        
        #     annotator.box_label(box=xyxy)

        # cv2.imwrite(f'/home/appuser/data/{os.path.basename(image)}', annotator.im.data)

    
    
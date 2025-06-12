import os
import cv2
import numpy as np
from glob import glob
from typing import List, Union
import matplotlib.pyplot as plt
from common_utils.annotate.core import Annotator
from common_utils.model.base import BaseModels, Detections
from common_utils.context_shift.core import ContextShift

def show_pipeline(
    images: List[Union[str, np.ndarray]],
    titles: List[str] = ["Original", "Segmented", "Contour + Attributes", "Filtered"],
    save_dir:str="",
):
    """
    Display a series of images side-by-side with arrows showing pipeline progression.

    Parameters:
    - images: List of image file paths or image arrays (BGR or RGB)
    - titles: List of titles to display above each image
    """
    assert len(images) == len(titles), "Images and titles must have the same length"

    n = len(images)
    fig, axes = plt.subplots(1, n, figsize=(12 * n, 18)) 

    if n == 1:
        axes = [axes]

    for i, (img, title) in enumerate(zip(images, titles)):
        if isinstance(img, str):
            img = cv2.imread(img)

        if img.shape[2] == 3:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        axes[i].imshow(img)
        axes[i].set_title(title, fontsize=12)
        axes[i].axis('off')

    plt.tight_layout()
    plt.savefig(save_dir)


if __name__ == "__main__":
    colors = [
        (0, 255, 0),
        (0, 165, 255),
        (0, 0, 255),
    ]

    model = BaseModels(
        weights='/media/appuser/rt_cvision/models/WasteImpurity_gml.pt',
        mlflow=False
    )
    images = sorted(glob('/media/appuser/rt_cvision/RTCVisionTestMedia/images/GML_gate06_bunker_2025-05-30_09-31-01_bb240deb-fd5f-49af-a85a-eed58a388f8d*.jpg'))

    for image in images:
        cv_image = cv2.imread(image)
        primary_detections = model.classify_one(
            image=cv_image,
            conf=0.25,
            mode='detect'
        )

        annotator_1 = Annotator(im=cv_image.copy())
        for i, xyxy in enumerate(primary_detections.xyxy):
            annotator_1.box_label(box=xyxy, color=colors[primary_detections.class_id[i]])

        context_shift = ContextShift(
            model_path="/media/appuser/rt_cvision/models/WasteMattressDet_V1.pt",
            conf_threshold=0.5,
            X=2,
        )

        primary_detections = primary_detections.to_dict()
        primary_detections['severity_level'] = [int(cls_id) + 1 for cls_id in primary_detections.get('class_id', [])]
        
        print(primary_detections)
        updated_primary = context_shift.integrate(
            primary_detections,
            full_image=cv_image,
        )

        print(updated_primary)
        annotator_2 = Annotator(im=cv_image.copy())
        for i, xyxy in enumerate(updated_primary["xyxy"]):
            annotator_2.box_label(
                box=xyxy, 
                color=colors[updated_primary["class_id"][i]], 
                label=updated_primary['context'][i] if updated_primary['context'][i] else ''
                )

        os.makedirs(f'/media/appuser/rt_cvision/RTCVisionTestMedia/runs', exist_ok=True)
        show_pipeline(
            images=[annotator_1.im.data, annotator_2.im.data],
            titles=["Original", "Context Added"],
            save_dir=f'/media/appuser/rt_cvision/RTCVisionTestMedia/runs/{os.path.basename(image)}',
        )
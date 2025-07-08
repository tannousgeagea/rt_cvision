import glob
import os
from collections import defaultdict
from typing import Dict, List, Optional, Tuple, Union

from torchvision.transforms import Compose

from common_utils.trackers.core.reid.dataset.base import TripletsDataset


def parse_market1501_dataset(data_dir: str) -> Dict[str, List[str]]:
    """Parse the [Market1501 dataset](https://paperswithcode.com/dataset/market-1501)
    to create a dictionary mapping tracker IDs to lists of image paths.

    Args:
        data_dir (str): The path to the Market1501 dataset.

    Returns:
        Dict[str, List[str]]: A dictionary mapping tracker IDs to lists of image paths.
    """
    image_files = glob.glob(os.path.join(data_dir, "*.jpg"))
    tracker_id_to_images = defaultdict(list)
    for image_file in image_files:
        tracker_id = os.path.basename(image_file).split("_")[0]
        tracker_id_to_images[tracker_id].append(image_file)
    return dict(tracker_id_to_images)


def get_market1501_dataset(
    data_dir: str,
    split_ratio: Optional[float] = None,
    random_state: Optional[Union[int, float, str, bytes, bytearray]] = None,
    shuffle: bool = True,
    transforms: Optional[Compose] = None,
) -> Union[TripletsDataset, Tuple[TripletsDataset, TripletsDataset]]:
    """Get the [Market1501 dataset](https://paperswithcode.com/dataset/market-1501).

    Args:
        data_dir (str): The path to the bounding box train/test directory of the
            [Market1501 dataset](https://paperswithcode.com/dataset/market-1501).
        split_ratio (Optional[float]): The ratio of the dataset to split into training
            and validation sets. If `None`, the dataset is returned as a single
            `TripletsDataset` object, otherwise the dataset is split into a tuple of
            training and validation `TripletsDataset` objects.
        random_state (Optional[Union[int, float, str, bytes, bytearray]]): The random
            state to use for the split.
        shuffle (bool): Whether to shuffle the dataset.
        transforms (Optional[Compose]): The transforms to apply to the dataset.

    Returns:
        Tuple[TripletsDataset, TripletsDataset]: A tuple of training and validation
            `TripletsDataset` objects.
    """
    tracker_id_to_images = parse_market1501_dataset(data_dir)
    dataset = TripletsDataset(tracker_id_to_images, transforms)
    if split_ratio is not None:
        train_dataset, validation_dataset = dataset.split(
            split_ratio=split_ratio, random_state=random_state, shuffle=shuffle
        )
        return train_dataset, validation_dataset
    return dataset

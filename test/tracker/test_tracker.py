import sys
import logging


sys.path.append(
    "/home/appuser/src/rt_cvision"
)

from common_utils.trackers import DeepSORTTracker, DeepSORTFeatureExtractor

import cv2
import numpy as np
from ultralytics import YOLO
from dataclasses import dataclass
from tqdm.auto import tqdm
from common_utils.trackers import SORTTracker
from common_utils.detection.core import Detections
from common_utils.annotate.core import Annotator
from typing import Callable, Generator, Optional, Tuple

CONFIDENCE_THRESHOLD = 0.3
NMS_THRESHOLD = 0.3

SOURCE_VIDEO_PATH = "/media/tracker/bikes-1280x720-2.mp4"
TARGET_VIDEO_PATH = "/media/tracker/bikes-1280x720-2-result.avi"
MAX_FRAMES = None
show_progress = True
progress_message = "Processing Video"

model = YOLO("yolo11s.pt")

# feature_extractor = DeepSORTFeatureExtractor.from_timm(
#     model_name="mobilenetv4_conv_small.e1200_r224_in1k")

# tracker = DeepSORTTracker(feature_extractor=feature_extractor)

tracker = SORTTracker()

@dataclass
class VideoInfo:
    """
    A class to store video information, including width, height, fps and
        total number of frames.

    Attributes:
        width (int): width of the video in pixels
        height (int): height of the video in pixels
        fps (int): frames per second of the video
        total_frames (Optional[int]): total number of frames in the video,
            default is None

    Examples:
        ```python
        import supervision as sv

        video_info = sv.VideoInfo.from_video_path(video_path=<SOURCE_VIDEO_FILE>)

        video_info
        # VideoInfo(width=3840, height=2160, fps=25, total_frames=538)

        video_info.resolution_wh
        # (3840, 2160)
        ```
    """

    width: int
    height: int
    fps: int
    total_frames: Optional[int] = None

    @classmethod
    def from_video_path(cls, video_path: str) -> 'VideoInfo':
        video = cv2.VideoCapture(video_path)
        if not video.isOpened():
            raise Exception(f"Could not open video at {video_path}")

        width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(video.get(cv2.CAP_PROP_FPS))
        total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
        video.release()
        return VideoInfo(width, height, fps, total_frames)

    @property
    def resolution_wh(self) -> Tuple[int, int]:
        return self.width, self.height

class VideoSink:
    """
    Context manager that saves video frames to a file using OpenCV.

    Attributes:
        target_path (str): The path to the output file where the video will be saved.
        video_info (VideoInfo): Information about the video resolution, fps,
            and total frame count.
        codec (str): FOURCC code for video format

    Example:
        ```python
        import supervision as sv

        video_info = sv.VideoInfo.from_video_path(<SOURCE_VIDEO_PATH>)
        frames_generator = sv.get_video_frames_generator(<SOURCE_VIDEO_PATH>)

        with sv.VideoSink(target_path=<TARGET_VIDEO_PATH>, video_info=video_info) as sink:
            for frame in frames_generator:
                sink.write_frame(frame=frame)
        ```
    """  # noqa: E501 // docs

    def __init__(self, target_path: str, video_info: VideoInfo, codec: str = "mp4v"):
        self.target_path = target_path
        self.video_info = video_info
        self.__codec = codec
        self.__writer = None

    def __enter__(self):
        try:
            self.__fourcc = cv2.VideoWriter_fourcc(*self.__codec)
        except TypeError as e:
            print(str(e) + ". Defaulting to mp4v...")
            self.__fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.__writer = cv2.VideoWriter(
            self.target_path,
            self.__fourcc,
            self.video_info.fps,
            self.video_info.resolution_wh,
        )
        return self

    def write_frame(self, frame: np.ndarray):
        """
        Writes a single video frame to the target video file.

        Args:
            frame (np.ndarray): The video frame to be written to the file. The frame
                must be in BGR color format.
        """
        self.__writer.write(frame)

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.__writer.release()


def _validate_and_setup_video(
    source_path: str, start: int, end: Optional[int], iterative_seek: bool = False
):
    video = cv2.VideoCapture(source_path)
    if not video.isOpened():
        raise Exception(f"Could not open video at {source_path}")
    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    if end is not None and end > total_frames:
        raise Exception("Requested frames are outbound")
    start = max(start, 0)
    end = min(end, total_frames) if end is not None else total_frames

    if iterative_seek:
        while start > 0:
            success = video.grab()
            if not success:
                break
            start -= 1
    elif start > 0:
        video.set(cv2.CAP_PROP_POS_FRAMES, start)

    return video, start, end

def get_video_frames_generator(
    source_path: str,
    stride: int = 1,
    start: int = 0,
    end: Optional[int] = None,
    iterative_seek: bool = False,
) -> Generator[np.ndarray, None, None]:
    """
    Get a generator that yields the frames of the video.

    Args:
        source_path (str): The path of the video file.
        stride (int): Indicates the interval at which frames are returned,
            skipping stride - 1 frames between each.
        start (int): Indicates the starting position from which
            video should generate frames
        end (Optional[int]): Indicates the ending position at which video
            should stop generating frames. If None, video will be read to the end.
        iterative_seek (bool): If True, the generator will seek to the
            `start` frame by grabbing each frame, which is much slower. This is a
            workaround for videos that don't open at all when you set the `start` value.

    Returns:
        (Generator[np.ndarray, None, None]): A generator that yields the
            frames of the video.

    Examples:
        ```python
        import supervision as sv

        for frame in sv.get_video_frames_generator(source_path=<SOURCE_VIDEO_PATH>):
            ...
        ```
    """
    video, start, end = _validate_and_setup_video(
        source_path, start, end, iterative_seek
    )
    frame_position = start
    while True:
        success, frame = video.read()
        if not success or frame_position >= end:
            break
        yield frame
        for _ in range(stride - 1):
            success = video.grab()
            if not success:
                break
        frame_position += stride
    video.release()

def callback(frame, _):
    results = model(frame)[0]
    detections = Detections.from_ultralytics(results)
    detections = tracker.update(detections)
    annotator = Annotator(im=frame)
    for detection_idx in range(len(detections)):
        xyxy = detections.xyxy[detection_idx].astype(int)
        annotator.box_label(
            box=xyxy,
            label=f"{detections.tracker_id[detection_idx]}" if detections.tracker_id is not None else ''
        )

    return annotator.im.data

def run():

    source_video_info = VideoInfo.from_video_path(video_path=SOURCE_VIDEO_PATH)
    video_frames_generator = get_video_frames_generator(
        source_path=SOURCE_VIDEO_PATH, end=None
    )

    print(source_video_info.total_frames)
    with VideoSink(target_path=TARGET_VIDEO_PATH, video_info=source_video_info, codec="XVID") as sink:
        total_frames = (
            min(source_video_info.total_frames, MAX_FRAMES)
            if MAX_FRAMES is not None
            else source_video_info.total_frames
        )
        for index, frame in enumerate(
            tqdm(
                video_frames_generator,
                total=total_frames,
                disable=not show_progress,
                desc=progress_message,
            )
        ):
            result_frame = callback(frame, index)
            sink.write_frame(frame=result_frame)
        else:
            for index, frame in enumerate(video_frames_generator):
                result_frame = callback(frame, index)
                sink.write_frame(frame=result_frame)


if __name__ == "__main__":
    run()
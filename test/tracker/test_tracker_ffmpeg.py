#!/usr/bin/env python3
"""
Efficient video processing system using FFmpeg for output generation.
Modular design with separate concerns for video I/O, processing, and tracking.
"""

import sys
import logging
import subprocess
import threading
import queue
from pathlib import Path
from dataclasses import dataclass
from typing import Callable, Generator, Optional, Tuple, Union
from contextlib import contextmanager
import signal
import os

import cv2
import numpy as np
from ultralytics import YOLO
from tqdm.auto import tqdm

# Add your custom modules path
sys.path.append("/home/appuser/src/rt_cvision")
from common_utils.trackers import SORTTracker
from common_utils.detection.core import Detections
from common_utils.annotate.core import Annotator


@dataclass
class VideoInfo:
    """Video information container with utility methods."""
    
    width: int
    height: int
    fps: int
    total_frames: Optional[int] = None
    
    @classmethod
    def from_video_path(cls, video_path: str) -> 'VideoInfo':
        """Extract video information from file."""
        video = cv2.VideoCapture(video_path)
        if not video.isOpened():
            raise FileNotFoundError(f"Could not open video at {video_path}")
        
        width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(video.get(cv2.CAP_PROP_FPS))
        total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
        
        video.release()
        return cls(width, height, fps, total_frames)
    
    @property
    def resolution_wh(self) -> Tuple[int, int]:
        """Get resolution as (width, height) tuple."""
        return self.width, self.height
    
    @property
    def aspect_ratio(self) -> float:
        """Calculate aspect ratio."""
        return self.width / self.height if self.height > 0 else 1.0


class VideoFrameGenerator:
    """Efficient video frame generator with seeking and stride support."""
    
    def __init__(self, source_path: str):
        self.source_path = Path(source_path)
        if not self.source_path.exists():
            raise FileNotFoundError(f"Video file not found: {source_path}")
    
    def generate_frames(self, 
                       stride: int = 1,
                       start: int = 0, 
                       end: Optional[int] = None,
                       max_frames: Optional[int] = None) -> Generator[Tuple[int, np.ndarray], None, None]:
        """
        Generate video frames with optional parameters.
        
        Yields:
            Tuple[int, np.ndarray]: (frame_index, frame)
        """
        video = cv2.VideoCapture(str(self.source_path))
        if not video.isOpened():
            raise RuntimeError(f"Failed to open video: {self.source_path}")
        
        try:
            total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
            end = min(end or total_frames, total_frames)
            
            # Set starting position
            if start > 0:
                video.set(cv2.CAP_PROP_POS_FRAMES, start)
            
            frame_count = 0
            current_position = start
            
            while current_position < end:
                success, frame = video.read()
                if not success:
                    break
                
                yield current_position, frame
                frame_count += 1
                
                # Check max_frames limit
                if max_frames and frame_count >= max_frames:
                    break
                
                # Skip frames according to stride
                for _ in range(stride - 1):
                    success = video.grab()
                    if not success:
                        return
                    current_position += 1
                    if current_position >= end:
                        return
                
                current_position += 1
                
        finally:
            video.release()


class FFmpegVideoSink:
    """
    Efficient video writer using FFmpeg subprocess with pipe input.
    Supports various codecs and quality settings.
    """
    
    def __init__(self, 
                 target_path: str,
                 video_info: VideoInfo,
                 codec: str = 'libx264',
                 crf: int = 18,
                 preset: str = 'medium',
                 pixel_format: str = 'yuv420p',
                 additional_args: Optional[list] = None):
        """
        Initialize FFmpeg video sink.
        
        Args:
            target_path: Output video file path
            video_info: Video dimensions and fps information
            codec: Video codec (libx264, libx265, etc.)
            crf: Constant Rate Factor (0-51, lower = better quality)
            preset: Encoding speed preset (ultrafast, fast, medium, slow, veryslow)
            pixel_format: Pixel format (yuv420p, yuv444p, etc.)
            additional_args: Additional FFmpeg arguments
        """
        self.target_path = Path(target_path)
        self.video_info = video_info
        self.codec = codec
        self.crf = crf
        self.preset = preset
        self.pixel_format = pixel_format
        self.additional_args = additional_args or []
        
        self._process = None
        self._frame_queue = queue.Queue(maxsize=10)  # Buffer frames
        self._writer_thread = None
        self._stop_writing = threading.Event()
        
        # Ensure output directory exists
        self.target_path.parent.mkdir(parents=True, exist_ok=True)
    
    def _build_ffmpeg_command(self) -> list:
        """Build FFmpeg command with all parameters."""
        cmd = [
            'ffmpeg',
            '-y',  # Overwrite output file
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{self.video_info.width}x{self.video_info.height}',
            '-pix_fmt', 'bgr24',
            '-r', str(self.video_info.fps),
            '-i', '-',  # Input from pipe
            '-c:v', self.codec,
            '-crf', str(self.crf),
            '-preset', self.preset,
            '-pix_fmt', self.pixel_format,
        ]
        
        # Add any additional arguments
        cmd.extend(self.additional_args)
        
        # Output file
        cmd.append(str(self.target_path))
        
        return cmd
    
    def _writer_worker(self):
        """Background thread worker for writing frames to FFmpeg."""
        while not self._stop_writing.is_set():
            try:
                frame = self._frame_queue.get(timeout=1.0)
                if frame is None:  # Poison pill
                    break
                self._process.stdin.write(frame.tobytes())
                self._frame_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                logging.error(f"Error in writer thread: {e}")
                break
    
    def __enter__(self):
        """Start FFmpeg process and writer thread."""
        cmd = self._build_ffmpeg_command()
        logging.info(f"Starting FFmpeg: {' '.join(cmd)}")
        
        try:
            self._process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
        except FileNotFoundError:
            raise RuntimeError("FFmpeg not found. Please install FFmpeg and ensure it's in PATH.")
        
        # Start background writer thread
        self._writer_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self._writer_thread.start()
        
        return self
    
    def write_frame(self, frame: np.ndarray):
        """
        Queue frame for writing to video.
        
        Args:
            frame: BGR image frame as numpy array
        """
        if self._process is None:
            raise RuntimeError("Video sink not started. Use within context manager.")
        
        # Ensure frame is in correct format and size
        if frame.shape[:2] != (self.video_info.height, self.video_info.width):
            frame = cv2.resize(frame, (self.video_info.width, self.video_info.height))
        
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            try:
                self._frame_queue.put(frame, timeout=5.0)
            except queue.Full:
                logging.warning("Frame queue full, dropping frame")
        else:
            raise ValueError(f"Invalid frame shape: {frame.shape}. Expected (H, W, 3)")
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Clean up FFmpeg process and threads."""
        # Signal writer thread to stop and send poison pill
        self._stop_writing.set()
        if self._frame_queue is not None:
            try:
                self._frame_queue.put(None, timeout=1.0)  # Poison pill
            except queue.Full:
                pass
        
        # Wait for writer thread
        if self._writer_thread and self._writer_thread.is_alive():
            self._writer_thread.join(timeout=5.0)
        
        # Close FFmpeg process
        if self._process:
            try:
                self._process.stdin.close()
                stdout, stderr = self._process.communicate(timeout=30)
                
                if self._process.returncode != 0:
                    logging.error(f"FFmpeg error: {stderr.decode()}")
                else:
                    logging.info(f"Video saved successfully: {self.target_path}")
                    
            except subprocess.TimeoutExpired:
                logging.warning("FFmpeg process timeout, terminating...")
                self._process.terminate()
                self._process.wait()


class VideoProcessor:
    """Main video processing orchestrator."""
    
    def __init__(self, 
                 model_path: str,
                 confidence_threshold: float = 0.3,
                 nms_threshold: float = 0.3):
        """
        Initialize video processor.
        
        Args:
            model_path: Path to YOLO model file
            confidence_threshold: Detection confidence threshold
            nms_threshold: Non-maximum suppression threshold
        """
        self.model = YOLO(model_path)
        self.tracker = SORTTracker()
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
    
    def process_frame(self, frame: np.ndarray, frame_index: int) -> np.ndarray:
        """
        Process a single frame with detection and tracking.
        
        Args:
            frame: Input frame
            frame_index: Frame number for reference
            
        Returns:
            Processed frame with annotations
        """
        # Run detection
        results = self.model(frame, conf=self.confidence_threshold, iou=self.nms_threshold)[0]
        detections = Detections.from_ultralytics(results)
        
        # Update tracker
        detections = self.tracker.update(detections)
        
        # Annotate frame
        annotator = Annotator(im=frame)
        
        if len(detections) > 0:
            for detection_idx in range(len(detections)):
                xyxy = detections.xyxy[detection_idx].astype(int)
                tracker_id = detections.tracker_id[detection_idx] if detections.tracker_id is not None else None
                
                label = f"{tracker_id} - gas canister" if tracker_id is not None else "gas canister"
                
                annotator.box_label(
                    box=xyxy,
                    label=label,
                    color=(0, 0, 255)
                )
        
        return annotator.im.data
    
    def process_video(self,
                     source_path: str,
                     target_path: str,
                     start_frame: int = 0,
                     end_frame: Optional[int] = None,
                     max_frames: Optional[int] = None,
                     stride: int = 1,
                     codec: str = 'libx264',
                     quality: str = 'high',
                     show_progress: bool = True,
                     progress_message: str = "Processing Video") -> bool:
        """
        Process entire video with detection and tracking.
        
        Args:
            source_path: Input video path
            target_path: Output video path
            start_frame: Starting frame number
            end_frame: Ending frame number (None for end of video)
            max_frames: Maximum number of frames to process
            stride: Frame stride (process every Nth frame)
            codec: Video codec for output
            quality: Quality setting ('low', 'medium', 'high', 'lossless')
            show_progress: Show progress bar
            progress_message: Progress bar description
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Get video information
            source_info = VideoInfo.from_video_path(source_path)
            logging.info(f"Processing video: {source_info.width}x{source_info.height}, {source_info.fps} fps, {source_info.total_frames} frames")
            
            # Create frame generator
            frame_generator = VideoFrameGenerator(source_path)
            
            # Calculate total frames for progress bar
            if max_frames:
                total_frames = min(max_frames, source_info.total_frames or 0)
            elif end_frame:
                total_frames = min(end_frame - start_frame, source_info.total_frames or 0)
            else:
                total_frames = source_info.total_frames or 0
            
            # Quality settings mapping
            quality_settings = {
                'lossless': {'crf': 0, 'preset': 'medium'},
                'high': {'crf': 18, 'preset': 'medium'},
                'medium': {'crf': 23, 'preset': 'fast'},
                'low': {'crf': 28, 'preset': 'fast'}
            }
            
            crf = quality_settings.get(quality, quality_settings['high'])['crf']
            preset = quality_settings.get(quality, quality_settings['high'])['preset']
            
            # Process video
            with FFmpegVideoSink(
                target_path=target_path,
                video_info=source_info,
                codec=codec,
                crf=crf,
                preset=preset
            ) as sink:
                
                frame_iter = frame_generator.generate_frames(
                    stride=stride,
                    start=start_frame,
                    end=end_frame,
                    max_frames=max_frames
                )
                
                if show_progress:
                    frame_iter = tqdm(
                        frame_iter,
                        total=total_frames // stride,
                        desc=progress_message,
                        unit='frames'
                    )
                
                processed_count = 0
                for frame_index, frame in frame_iter:
                    try:
                        processed_frame = self.process_frame(frame, frame_index)
                        sink.write_frame(processed_frame)
                        processed_count += 1
                        
                    except Exception as e:
                        logging.error(f"Error processing frame {frame_index}: {e}")
                        continue
                
                logging.info(f"Successfully processed {processed_count} frames")
                return True
                
        except Exception as e:
            logging.error(f"Video processing failed: {e}")
            return False


def main():
    """Main execution function with configuration."""
    # Configuration
    config = {
        'model_path': "/media/GasCanisterDet_V1.pt",
        'source_video': "/media/images/gas_canister/gas_canister-fps3.mp4",
        'target_video': "/media/images/gas_canister/gas_canister-fps3-result.mp4",
        'confidence_threshold': 0.3,
        'nms_threshold': 0.3,
        'max_frames': None,
        'stride': 1,
        'start_frame': 0,
        'end_frame': None,
        'codec': 'libx264',  # or 'libx265' for better compression
        'quality': 'high',   # 'low', 'medium', 'high', 'lossless'
        'show_progress': True,
        'progress_message': "Processing Video"
    }
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    # Initialize processor
    try:
        processor = VideoProcessor(
            model_path=config['model_path'],
            confidence_threshold=config['confidence_threshold'],
            nms_threshold=config['nms_threshold']
        )
        
        # Process video
        success = processor.process_video(
            source_path=config['source_video'],
            target_path=config['target_video'],
            start_frame=config['start_frame'],
            end_frame=config['end_frame'],
            max_frames=config['max_frames'],
            stride=config['stride'],
            codec=config['codec'],
            quality=config['quality'],
            show_progress=config['show_progress'],
            progress_message=config['progress_message']
        )
        
        if success:
            print(f"✅ Video processing completed successfully!")
            print(f"Output saved to: {config['target_video']}")
        else:
            print("❌ Video processing failed!")
            sys.exit(1)
            
    except Exception as e:
        logging.error(f"Failed to initialize processor: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
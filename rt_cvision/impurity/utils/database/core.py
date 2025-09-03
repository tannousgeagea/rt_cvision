
import os
import io
import cv2
import uuid
import django
django.setup()
import numpy as np
import logging
from PIL import Image as PILImage
from typing import Optional, Dict
from datetime import timedelta
from typing import Union
from metadata.models import Tag
from datetime import datetime, timezone
from data_reader.models import Image, ImageTag
from impurity.models import Impurity, ImpurityTag
from common_utils.detection.core import Detections
from common_utils import DATETIME_FORMAT_IN_FILENAME
from django.core.files.base import ContentFile
from django.utils.timezone import now
from django.db import transaction
from django.utils import timezone as django_timezone
from django.contrib.auth import get_user_model
User = get_user_model()

logger = logging.getLogger(__name__)

class ImageSaveError(Exception):
    """Custom exception for image save operations"""
    pass

def compress_image(cv_image, quality:int=65):
    pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
    img_size = pil_image.size
    compressed_io = io.BytesIO()
    pil_image.convert("RGB").save(
        compressed_io,
        format="JPEG",
        optimize=True,
        quality=quality
    )

    compressed_io.seek(0)
    return compressed_io.read(), img_size

class DatabaseManager:
    def __init__(self, config:dict = {}) -> None:
        self.config = config
        self.user = User.objects.filter(email=os.getenv('DJANGO_SUPERUSER_EMAIL')).first()
        self.tenant = self.config.get("tenant") or {}
        self.entity = self.tenant.get('entity') or {}
        self.sensorbox = self.tenant.get('sensor_box') or {}
        self.prefix =(
            f"{self.tenant.get('tenant_name', 'WasteAnt')}_"
            f"{self.entity.get('uid', 'gate')}_"
            f"{self.sensorbox.get('location', 'front')}_"
        )

    def save_image_v2(
        self, 
        cv_image: np.ndarray, 
        event_uid: Optional[str] = None, 
        filename: Optional[str] = None,
        validate_image: bool = True,
        max_file_size: int = 50 * 1024 * 1024,  # 50MB default limit
        expires_in: Union[int, timedelta, None] = timedelta(days=7),
        auto_expire: bool = True
    ) -> Image | None:
        """
        Save a CV2 image to the database with improved error handling and validation.
        
        Args:
            cv_image: OpenCV image array (numpy.ndarray)
            event_uid: Optional unique identifier for the event
            filename: Optional custom filename
            validate_image: Whether to validate the image before saving
            max_file_size: Maximum allowed file size in bytes
        
        Returns:
            Image object if successful, None if failed
        
        Raises:
            ImageSaveError: If image saving fails due to validation or processing errors
            ValueError: If input parameters are invalid
        """
        
        # Input validation
        if cv_image is None:
            raise ValueError("cv_image cannot be None")
        
        if not isinstance(cv_image, np.ndarray):
            raise ValueError("cv_image must be a numpy ndarray")
        
        if validate_image and len(cv_image.shape) not in [2, 3]:
            raise ValueError("cv_image must be a 2D or 3D array (grayscale or color)")
        
        # Generate unique identifiers
        event_uid = event_uid or str(uuid.uuid4())
        current_time = django_timezone.now()
        
        expires_at = None
        if auto_expire:
            if isinstance(expires_in, int):
                expires_at = current_time + timedelta(seconds=expires_in)
            elif isinstance(expires_in, timedelta):
                expires_at = current_time + expires_in
            elif expires_in is not None:
                raise ValueError("expires_in must be int (seconds), timedelta, or None")

        if filename is None:
            timestamp_str = current_time.strftime(DATETIME_FORMAT_IN_FILENAME)
            filename = f"{self.prefix}{timestamp_str}_{event_uid}.jpg"
        
        # Validate filename uniqueness (optional - depends on requirements)
        if Image.objects.filter(image_id=event_uid).exists():
            logger.warning(f"Image with event_uid {event_uid} already exists")
            # You can choose to: raise error, generate new UUID, or overwrite
            event_uid = str(uuid.uuid4())
        
        try:
            with transaction.atomic():
                # Compress image and get metadata
                file_content, img_dimensions = compress_image(cv_image=cv_image)
                
                if file_content is None:
                    raise ImageSaveError("Image compression failed")
                
                # Validate file size
                file_size = len(file_content)
                if file_size > max_file_size:
                    raise ImageSaveError(f"Compressed image size ({file_size} bytes) exceeds limit ({max_file_size} bytes)")
                
                # Create Image object with all metadata
                image = Image(
                    image_id=event_uid,
                    image_name=filename,
                    image_format='JPEG',
                    image_size=file_size,  # Set the file size in bytes
                    width=img_dimensions[0] if img_dimensions else None,
                    height=img_dimensions[1] if img_dimensions else None,
                    timestamp=current_time,
                    sensorbox_id=self.sensorbox.get('id'),
                    source=getattr(self, 'source', None),  # Add source if available
                    is_processed=False,  # Explicitly set processing status
                    expires_at=expires_at,
                )
                
                # Save file and create database record in one operation
                image.image_file.save(
                    filename,
                    ContentFile(file_content),
                    save=True  # This will save the Image object automatically
                )
                
                logger.info(f"Successfully saved image {event_uid} ({file_size} bytes)")
                return image
                
        except Exception as err:
            # Log the full error for debugging
            logger.error(f"Failed to save image {event_uid}: {str(err)}", exc_info=True)
            
            # Re-raise with appropriate exception type
            if isinstance(err, (ValueError, ImageSaveError)):
                raise
            else:
                raise ImageSaveError(f"Database error while saving image: {str(err)}") from err 
            
            
    def save_image(self, cv_image:np.ndarray, event_uid:Optional[str] = None, filename:Optional[str] = None) -> Image | None:
        try:
            event_uid = event_uid or str(uuid.uuid4())
            filename = filename or f"{self.prefix}{datetime.now().strftime(DATETIME_FORMAT_IN_FILENAME)}_{event_uid}.jpg"
            image = Image(
                image_id=event_uid,
                image_name=filename,
                image_format='JPEG',
                timestamp=datetime.now(tz=timezone.utc),
                sensorbox_id=self.sensorbox.get('id'),
            )
            file_content, img_size = compress_image(cv_image=cv_image)
            image.width, image.height = img_size
            image.image_file.save(
                filename,
                ContentFile(file_content)
                )
            image.save()
            
            return image
        except Exception as err:
            raise ValueError(f"[Database][Image] Error: {err}")

    def save_detections(self, detections:Detections, cv_image):
        try:
            n_detections = len(detections)
            if not n_detections:
                return

            image = self.save_image_v2(cv_image=cv_image)
            data = detections.data
            for detection_idx in range(n_detections):
                wi = Impurity.objects.create(
                    image=image,
                    object_uid=detections.uid[detection_idx] if detections.uid is not None else str(uuid.uuid4()),
                    timestamp=now(),
                    confidence_score=detections.confidence[detection_idx] if detections.confidence is not None else 0.5,
                    class_id=detections.class_id[detection_idx] if detections.class_id is not None else 0,
                    object_coordinates=detections.xyxyn[detection_idx].tolist(),
                    object_length=detections.object_length[detection_idx] if detections.object_length is not None else None,
                    meta_info={
                        k: v[detection_idx].tolist() if isinstance(v[detection_idx], np.ndarray) else v[detection_idx] for k, v in data.items()
                    }
                )

                tag_names = set()
                for name, value in data.items():
                    vi = value[detection_idx]
                    if vi:
                        if isinstance(vi, list) or isinstance(vi, np.ndarray):
                            for vi_i in vi:
                                if vi_i:
                                    tag_names.add(vi_i)
                        elif isinstance(vi, str):
                            tag_names.add(vi)

                logging.info(f"[Database] Tags: {tag_names}")
                for name in tag_names:
                    logging.info(f"[Database] Tag: {name}")
                    tag, _ = Tag.objects.get_or_create(name=name)
                    ImpurityTag.objects.get_or_create(
                        impurity=wi,
                        tag=tag,
                        tagged_by=self.user,
                        source='model'
                    )
                    
                    ImageTag.objects.get_or_create(
                        image=image,
                        tag=tag,
                        tagged_by=self.user,
                        source='model',
                    )

        except Exception as err:
            raise ValueError(f"[Database][Detection] Error: {err}")

    def save(self, data:Dict):
        try:
            assert 'cv_image' in data, "cv_image not found in data"
            assert not data['cv_image'] is None, "Image is None"
            assert "pdetections" in data, "detections not found in data"

            cv_image = data['cv_image']
            detections = data['pdetections']
            self.save_detections(detections=Detections.from_dict(detections), cv_image=cv_image)
        except Exception as err:
            raise ValueError(f"[Database] Error: {err}")





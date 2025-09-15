# django_ollama_integration.py
"""
Django Integration for Ollama API Service
Handles Impurity model events and processes waste analysis using vision-language models

Components:
1. Redis queue system for event handling
2. Celery tasks for async processing
3. Image processing with bounding box visualization
4. Ollama API client integration
5. Response parsing and storage
"""

import os
import json
import redis
import requests
import base64
from PIL import Image, ImageDraw, ImageFont
from io import BytesIO
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from datetime import datetime
import logging

# Django imports
from django.conf import settings
from django.core.files.storage import default_storage

# Configure logging
logger = logging.getLogger(__name__)

# Configuration
OLLAMA_API_URL = os.getenv('OLLAMA_API_URL', 'http://server2.learning.test.want:11434')
OLLAMA_API_KEY = os.getenv('OLLAMA_API_KEY')  # Optional
VISION_MODEL = os.getenv('VISION_MODEL', 'llama3.2-vision:latest')

ALLOWED_INSTANCE_TYPES = [
    'mattress', 'sofa', 'chair', 'table', 'cabinet', 'rug', 'duvet', 
    'bed sheet', 'pillow', 'fabric', 'cardboard', 'paper', 'plastic bag',
    'plastic bottle', 'glass bottle', 'metal can', 'wooden pallet', 
    'gas canister', 'fire extinguisher', 'battery', 'electronics', 
    'pipe', 'metal object', 'wood plank', 'brick/concrete', 'rubber item', 
    'organic waste', 'tire', 'cable', 'container', 'packaging', 'textile', 
    'other'
]

ALLOWED_CONDITIONS = [
    'intact', 'damaged', 'torn', 'crushed', 'weathered', 'new', 'worn'
]


# ===================
# DATA CLASSES
# ===================

@dataclass
class BoundingBox:
    """Represents a bounding box with normalized coordinates"""
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    
    @classmethod
    def from_coordinates(cls, coordinates: List[float]) -> 'BoundingBox':
        """Create from [x_min, y_min, x_max, y_max] list"""
        return cls(
            x_min=coordinates[0],
            y_min=coordinates[1], 
            x_max=coordinates[2],
            y_max=coordinates[3]
        )
    
    def to_pixel_coordinates(self, image_width: int, image_height: int) -> Tuple[int, int, int, int]:
        """Convert normalized coordinates to pixel coordinates"""
        return (
            int(self.x_min * image_width),
            int(self.y_min * image_height),
            int(self.x_max * image_width),
            int(self.y_max * image_height)
        )


@dataclass
class WasteAnalysisResult:
    """Structured result from waste analysis"""
    instance_type: str
    color: str
    material: str
    visibility: str
    size: str
    location: str
    condition: str
    confidence: float
    reasoning: str
    context_used: str
    
    @classmethod
    def from_api_response(cls, response_data: Dict[str, Any]) -> 'WasteAnalysisResult':
        """Create from API response"""
        obj_data = response_data.get('object', {})
        return cls(
            instance_type=obj_data.get('instance_type', 'unknown'),
            color=obj_data.get('color', 'unknown'),
            material=obj_data.get('material', 'unknown'),
            visibility=obj_data.get('visibility', 'unknown'),
            size=obj_data.get('size', 'unknown'),
            location=obj_data.get('location', 'unknown'),
            condition=obj_data.get('condition', 'unknown'),
            confidence=float(obj_data.get('confidence', 0.0)),
            reasoning=obj_data.get('reasoning', "unknown"),
            context_used=obj_data.get('context_used', 'unknown'),
        )
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for storage"""
        return {
            'instance_type': self.instance_type,
            'color': self.color,
            'material': self.material,
            'visibility': self.visibility,
            'size': self.size,
            'location': self.location,
            'condition': self.condition,
            'confidence': self.confidence,
            'reasoning': self.reasoning,
            'context_used': self.context_used,
        }

# ===================
# IMAGE PROCESSING
# ===================

class ImageProcessor:
    """Handles image processing and bounding box visualization"""
    
    @staticmethod
    def load_image_from_model(image_model) -> Image.Image:
        """Load PIL Image from Django ImageField"""
        try:
            # Handle different storage backends
            if hasattr(image_model.image_file, 'url'):
                # For file-based storage
                image_path = image_model.image_file.path
                return Image.open(image_path)
            else:
                # For cloud storage (S3, etc.)
                image_file = default_storage.open(image_model.image_file.name)
                return Image.open(image_file)
        except Exception as e:
            logger.error(f"Failed to load image {image_model.pk}: {e}")
            raise
    
    @staticmethod
    def draw_bounding_box(image: Image.Image, bbox: BoundingBox, color: str = 'red', width: int = 3) -> Image.Image:
        """Draw bounding box on image"""
        # Create a copy to avoid modifying original
        img_copy = image.copy()
        draw = ImageDraw.Draw(img_copy)
        
        # Convert to pixel coordinates
        img_width, img_height = image.size
        pixel_coords = bbox.to_pixel_coordinates(img_width, img_height)
        
        # Draw rectangle
        draw.rectangle(pixel_coords, outline=color, width=width)
        corner_size = 10
        corners = [
            (pixel_coords[0], pixel_coords[1]), (pixel_coords[2], pixel_coords[1]), 
            (pixel_coords[0], pixel_coords[3]), (pixel_coords[2], pixel_coords[3])
        ]
        
        for corner in corners:
            draw.rectangle([
                (corner[0] - corner_size, corner[1] - corner_size),
                (corner[0] + corner_size, corner[1] + corner_size)
            ], fill=color)


        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
        except:
            font = ImageFont.load_default()

        draw.text((pixel_coords[0], pixel_coords[1] - 25), "TARGET OBJECT", 
                 fill=color, font=font)


        img_copy.save("test.png")
        return img_copy
    
    def extract_bbox_region(self, image: Image.Image, bbox: BoundingBox, color: str = 'red',
                           padding: float = 0.1) -> Tuple[Image.Image, dict]:
        """
        Extract and analyze only the bounding box region
        More accurate but loses context
        """
        width, height = image.size
        pixel_coords = bbox.to_pixel_coordinates(width, height)
        
        # Convert to pixel coordinates with optional padding
        x_min = max(0, int((pixel_coords[0] - padding * width)))
        y_min = max(0, int((pixel_coords[1] - padding * height)))
        x_max = min(width, int((pixel_coords[2] + padding * width)))
        y_max = min(height, int((pixel_coords[3] + padding * height)))
        
        # Crop the region
        cropped = image.crop((x_min, y_min, x_max, y_max))
        
        # Return cropped image and metadata
        crop_info = {
            "original_size": (width, height),
            "crop_coords": (x_min, y_min, x_max, y_max),
            "crop_size": cropped.size,
            "padding_used": padding
        }
        

        cropped.save("test.png")
        return cropped, crop_info
    
    @staticmethod
    def image_to_base64(image: Image.Image, format: str = 'JPEG') -> str:
        """Convert PIL Image to base64 string"""
        buffer = BytesIO()
        # Convert to RGB if necessary (for JPEG)
        if format == 'JPEG' and image.mode in ('RGBA', 'LA', 'P'):
            image = image.convert('RGB')
        
        image.save(buffer, format=format)
        image_data = buffer.getvalue()
        return base64.b64encode(image_data).decode('utf-8')

# ===================
# OLLAMA API CLIENT
# ===================

class OllamaAPIClient:
    """Client for communicating with Ollama API service"""
    
    def __init__(self, base_url: str = OLLAMA_API_URL, api_key: str = OLLAMA_API_KEY):
        self.base_url = base_url.rstrip('/')
        self.headers = {'Content-Type': 'application/json'}
        if api_key:
            self.headers['Authorization'] = f'Bearer {api_key}'
    
    def analyze_waste_object(
        self,
        image_base64: str,
        coordinates: List[float],
        model: str = VISION_MODEL,
        mode: str = "visual_bbox",
        context: dict = None,
    ) -> Dict[str, Any]:
        """
        Analyze waste object using vision-language model
        
        Args:
            image_base64: Base64 encoded image
            coordinates: Bounding box coordinates [x_min, y_min, x_max, y_max]
            model: Vision model to use
            
        Returns:
            API response with analysis results
        """
        prompt = self._create_analysis_prompt(mode, coordinates, context=context)
        
        print(prompt)
        payload = {
            'model': model,
            'prompt': prompt,
            'images': [image_base64],
            'format': 'json',  # Request JSON response
            'stream': False,
            'keep_alive': 0,
            'options': {
                'temperature': 0.1,  # Lower temperature for more consistent results
                'top_p': 0.9
            }
        }
        
        try:
            response = requests.post(
                f'{self.base_url}/api/generate',
                headers=self.headers,
                json=payload,
                timeout=120  # 2 minute timeout for vision analysis
            )
            response.raise_for_status()
            return response.json()
            
        except requests.exceptions.RequestException as e:
            logger.error(f"Ollama API request failed: {e}")
            raise
    
    def _create_analysis_prompt(self, mode:str="visual_bbox", coordinates: List[float]=None, context:dict=None) -> str:
        """Create the analysis prompt with bounding box coordinates"""
        return self._build_prompt(mode=mode, coords=coordinates, context=context)

    def _format_context_block(self, context: dict) -> str:
        """Format additional context into bullet points"""
        
        if context is None:
            return ""
        
        items = []
        for key, value in context.items():
            items.append(f"- {key}: {value}")

        # if context.get("object_length") is not None:
        #     items.append(f"- Object length: {context['object_length']} cm")
        # if context.get("detection_confidence") is not None:
        #     items.append(f"- Detection confidence: {context['detection_confidence']:.2f}")
        # if context.get("class_id") is not None:
        #     class_name = context.get("detected_class", "unknown")
        #     items.append(f"- Detected class: {class_name} (ID: {context['class_id']})")
        # if context.get("timestamp"):
        #     items.append(f"- Detection time: {context['timestamp']}")
        # if context.get("object_uid"):
        #     items.append(f"- Object UID: {context['object_uid']}")
        # if context.get("additional_notes"):
        #     items.append(f"- Notes: {context['additional_notes']}")
        
        return "Additional context:\n" + "\n".join(items) if items else ""

    def _build_prompt(self, mode:str = "visual_bbox", coords=None, context:dict=None) -> str:
        CONTEXT_BLOCK = self._format_context_block(context)
        BASE_PROMPT = f"""
            You are a vision-language AI trained to analyze waste material in industrial settings.  

            Your task is to analyze a target waste object and return detailed semantic and visual properties for it.  
            Focus ONLY on the target object. Ignore everything else.  

            If the object is ambiguous or partially visible, output instance_type="other" and confidence<=0.5.  

            Return STRICT JSON only. If any field is uncertain, lower confidence.  

            Allowed instance_type values:
            {ALLOWED_INSTANCE_TYPES}

            Allowed condition values:
            {ALLOWED_CONDITIONS}
            
            {CONTEXT_BLOCK}
            
            Describe the object with the following properties:

            - "instance_type": The most appropriate label from the allowed list.  
            - "color": The dominant color or meaningful color pattern (e.g., "blue and grey", "rusted metal").  
            - "material": The inferred material type (e.g., plastic, metal, foam, wood, fabric, rubber, composite).  
            - "visibility": One of 'fully visible', 'partially occluded', or 'heavily occluded'.  
            - "size": Relative to the given input (either full image ROI or cropped image) — 'small', 'medium', or 'large'.  
            - "location": Spatial location within the input (e.g., 'top right', 'center').  
            - "condition": The current state of the object.
            - "confidence": Your certainty (float between 0.0 and 1.0) that the interpretation is correct.  
            - "reasoning": A short explanation of why you assigned this instance_type, material, color, etc.  
            - "context_used": What context fields (length, detection_confidence, etc.) were used. Explicitly include "object_length" when determining size.

            Output format:

            {{
                "object": {{
                    "instance_type": "<instance_type>",
                    "color": "<dominant_color>",
                    "material": "<inferred_material>",
                    "visibility": "<visibility_level>",
                    "size": "<object_size>",
                    "location": "<spatial_position>",
                    "condition": "<condition>",
                    "confidence": <confidence_score>,
                    "reasoning": "<short explanation>"
                    "context_used": "<fields that influenced the decision>"
                }}
            }}

        """


        BBOX_MODE = """You are analyzing waste in a bunker.
        The RED BOUNDING BOX shows the exact object to analyze.
        Original coordinates: {coordinates} (x_min, y_min, x_max, y_max).
        Focus ONLY on the object inside the red box. Ignore everything outside the box.
        """

        CROP_MODE = """You are given a single cropped image that contains ONLY the target object.
        Do not use any scene context.
        """
        if mode == "visual_bbox":
            return BBOX_MODE.format(coordinates=coords) + "\n\n" + BASE_PROMPT
        elif mode == "cropped":
            return CROP_MODE + "\n\n" + BASE_PROMPT
        else:
            raise ValueError("mode must be 'visual_bbox' or 'cropped'")

    def check_health(self) -> bool:
        """Check if Ollama API service is available"""
        try:
            response = requests.get(f'{self.base_url}/api/tags', timeout=10)
            return response.status_code == 200
        except:
            return False

def process_impurity(impurity_id: int):
    """
    Celery task to process impurity analysis
    
    Args:
        impurity_id: Primary key of the Impurity instance
    """
    import django
    django.setup()
    from data_reader.models import ImageTag
    from metadata.models import Tag, TagGroup
    from impurity.models import Impurity, ImpurityTag  # Adjust import path
    from django.contrib.auth import get_user_model
    User = get_user_model()
    
    try:
        logger.info(f"Starting processing for impurity {impurity_id}")
        
        # Get impurity instance
        try:
            impurity = Impurity.objects.select_related('image').get(pk=impurity_id)
        except Impurity.DoesNotExist:
            logger.error(f"Impurity {impurity_id} not found")
            return
        
        # Skip if already processed
        if impurity.is_processed:
            logger.info(f"Impurity {impurity_id} already processed")
            return
        
        # Validate required data
        if not impurity.image or not impurity.object_coordinates:
            logger.error(f"Impurity {impurity_id} missing required data")
            return
        
        # Initialize clients
        processor = ImageProcessor()
        api_client = OllamaAPIClient()
        
        # Check API health
        if not api_client.check_health():
            logger.error("Ollama API is not available")
            raise Exception("Ollama API service unavailable")
        
        # Load and process image
        original_image = processor.load_image_from_model(impurity.image)
        
        # Create bounding box
        bbox = BoundingBox.from_coordinates(impurity.object_coordinates)
        
        # Draw bounding box on image
        # image_with_bbox, _ = processor.extract_bbox_region(original_image, bbox)
        image_with_bbox = processor.draw_bounding_box(original_image, bbox)
        
        # cropped, _ = processor.extract_bbox_region(image=original_image, bbox=bbox)
        # Convert to base64
        image_base64 = processor.image_to_base64(image_with_bbox)
        
        context = {
            "Object Length": f"{impurity.object_length * 100} cm" if impurity.object_length else "unknown",
            "Detection Confidence": impurity.confidence_score,
            "Detection Time": impurity.timestamp,
            # "Detected Class": impurity.meta_info.get('class_name') if  impurity.meta_info is not None else 'unknown',
            # "Attributes": impurity.meta_info.get('attributes') if  impurity.meta_info is not None else 'unknown',
        }

        # Analyze with Ollama
        api_response = api_client.analyze_waste_object(
            image_base64=image_base64,
            coordinates=impurity.object_coordinates,
            mode="visual_bbox",
            context=context,
        )
        
        # Parse response
        try:
            # The response should contain JSON in the 'response' field
            response_text = api_response.get('response', '{}')
            analysis_data = json.loads(response_text)
            
            # Create structured result
            result = WasteAnalysisResult.from_api_response(analysis_data)
            
        except (json.JSONDecodeError, KeyError) as e:
            logger.error(f"Failed to parse API response for impurity {impurity_id}: {e}")
            # Create fallback result
            result = WasteAnalysisResult(
                instance_type='unknown',
                color='unknown', 
                material='unknown',
                visibility='unknown',
                size='unknown',
                location='unknown',
                condition='unknown',
                confidence=0.0,
                reasoning='unknown',
                context_used='unknown',
            )
        
        # Update impurity with results
        if impurity.meta_info is None:
            impurity.meta_info = {
                'analysis': result.to_dict(),
                'api_response': api_response,
                'processed_at': datetime.now().isoformat(),
                'model_used': VISION_MODEL,
                'processing_version': '1.0'
            }
        else:
            impurity.meta_info.update({
                'analysis': result.to_dict(),
                'api_response': api_response,
                'processed_at': datetime.now().isoformat(),
                'model_used': VISION_MODEL,
                'processing_version': '1.0'
            })
        impurity.is_processed = True
        impurity.save()

        tag_groups = TagGroup.objects.all()
        for tag_group in tag_groups:
            if tag_group.name in result.to_dict().keys():
                value = result.to_dict()[tag_group.name]
                tag, _ = Tag.objects.get_or_create(name=value)
                if not tag:
                    tag, _ = Tag.objects.get_or_create(name=value, group=tag_group)
                ImpurityTag.objects.get_or_create(
                    impurity=impurity,
                    tag=tag,
                    tagged_by=User.objects.filter(email=os.getenv('DJANGO_SUPERUSER_EMAIL')).first(),
                    source='model'
                )
                
                ImageTag.objects.get_or_create(
                    image=impurity.image,
                    tag=tag,
                    tagged_by=User.objects.filter(email=os.getenv('DJANGO_SUPERUSER_EMAIL')).first(),
                    source='model',
                )
        
        print(f"Successfully processed impurity {impurity_id}")
        print(f"Analysis result: {result.instance_type} ({result.confidence:.2f} confidence)")
        
        
    except Exception as exc:
        logger.error(f"Error processing impurity {impurity_id}: {exc}")
        
        # Retry logic
        # if self.request.retries < self.max_retries:
        #     logger.info(f"Retrying impurity {impurity_id} (attempt {self.request.retries + 1})")
        #     raise self.retry(exc=exc)
        # else:
        #     logger.error(f"Max retries exceeded for impurity {impurity_id}")
        #     # Mark as failed
        #     try:
        #         impurity = Impurity.objects.get(pk=impurity_id)
        #         impurity.meta_info = {
        #             'error': str(exc),
        #             'failed_at': datetime.now().isoformat(),
        #             'max_retries_exceeded': True
        #         }
        #         impurity.save()
        #     except:
        #         pass

def process_pending_impurities():
    """Process all pending impurities in batch"""
    import django
    django.setup()
    from impurity.models import Impurity
    
    pending_impurities = Impurity.objects.filter(is_processed=False)
    
    logger.info(f"Found {pending_impurities.count()} pending impurities")
    
    for impurity in pending_impurities:
        try:
            process_impurity(impurity.pk)
        except Exception as e:
            logger.error(f"Failed to queue impurity {impurity.pk}: {e}")

def re_process_impurities():
    """Process all pending impurities in batch"""
    import django
    django.setup()
    from impurity.models import Impurity
    
    processed_impurities = Impurity.objects.filter(is_processed=True)
    
    print(f"Found {processed_impurities.count()} processed impurities")
    
    for impurity in processed_impurities:
        try:
            process_impurity(impurity.pk)
        except Exception as e:
            logger.error(f"Failed to queue impurity {impurity.pk}: {e}")



if __name__ == "__main__":
    impurity_id = 5039
    process_impurity(impurity_id)
    # process_pending_impurities()

import os
import json
import logging
from datetime import datetime
from celery import shared_task

from common_utils.context.core import ImageProcessor, OllamaAPIClient, BoundingBox, WasteAnalysisResult

logger = logging.getLogger(__name__)

OLLAMA_API_URL = os.getenv('OLLAMA_API_URL', 'http://server2.learning.test.want:11434')
OLLAMA_API_KEY = os.getenv('OLLAMA_API_KEY')  # Optional
VISION_MODEL = os.getenv('VISION_MODEL', 'llama3.2-vision:latest')
MODE = "visual_bbox"


@shared_task(
    bind=True,
    autoretry_for=(Exception,),
    retry_backoff=True,
    retry_kwargs={"max_retries": 5},
    ignore_result=True,
    name='context:execute'
)
def execute(self, impurity_id: int):
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
        
        if MODE == "visual_bbox":
        # Draw bounding box on image
            image_with_bbox = processor.draw_bounding_box(original_image, bbox)
        elif MODE == "cropped":
            image_with_bbox, _ = processor.extract_bbox_region(original_image, bbox)
        else:
            raise ValueError("mode must be 'visual_bbox' or 'cropped'")
        
        # Convert to base64
        image_base64 = processor.image_to_base64(image_with_bbox)
        
        context = {
            "Object Length": f"{impurity.object_length * 100} cm" if impurity.object_length else "unknown",
            "Detection Confidence": impurity.confidence_score,
            "Detection Time": impurity.timestamp,
        }

        # Analyze with Ollama
        api_response = api_client.analyze_waste_object(
            image_base64=image_base64,
            coordinates=impurity.object_coordinates,
            mode=MODE,
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
            result_json = result.to_dict()
            if tag_group.name in result_json.keys():
                value = result_json[tag_group.name]
                confidence = result_json['confidence']
                tag = Tag.objects.filter(name=value).first()
                if not tag:
                    tag, _ = Tag.objects.get_or_create(name=value, group=tag_group)
                ImpurityTag.objects.get_or_create(
                    impurity=impurity,
                    tag=tag,
                    confidence=float(confidence),
                    tagged_by=User.objects.filter(email=os.getenv('DJANGO_SUPERUSER_EMAIL')).first(),
                    source='model'
                )
                
                ImageTag.objects.get_or_create(
                    image=impurity.image,
                    tag=tag,
                    confidence=float(confidence),
                    tagged_by=User.objects.filter(email=os.getenv('DJANGO_SUPERUSER_EMAIL')).first(),
                    source='model',
                )
        
        logger.info(f"Successfully processed impurity {impurity_id}")
        logger.info(f"Analysis result: {result.instance_type} ({result.confidence:.2f} confidence)")
        
    except Exception as exc:
        logger.error(f"Error processing impurity {impurity_id}: {exc}")
        
        # Retry logic
        if self.request.retries < self.max_retries:
            logger.info(f"Retrying impurity {impurity_id} (attempt {self.request.retries + 1})")
            raise self.retry(exc=exc)
        else:
            logger.error(f"Max retries exceeded for impurity {impurity_id}")
            # Mark as failed
            try:
                impurity = Impurity.objects.get(pk=impurity_id)
                if impurity.meta_info is None:
                    impurity.meta_info = {
                        'error': str(exc),
                        'failed_at': datetime.now().isoformat(),
                        'max_retries_exceeded': True
                    }
                else: 
                    impurity.meta_info.update({
                        'error': str(exc),
                        'failed_at': datetime.now().isoformat(),
                        'max_retries_exceeded': True
                    })
                impurity.save()
            except:
                pass
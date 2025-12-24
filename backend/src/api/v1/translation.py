from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
import logging

from ...models.translation import TranslationRequest, TranslationResponse, ErrorResponse, TranslationEngine
from ...services.translation_service import TranslationService

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize the translation service
translation_service = TranslationService(cache_ttl_seconds=3600)


@router.post("/translate", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    """
    Translate text to the requested target language.
    Uses open-source models with optional LLM fallback.
    """
    try:
        # Validate the request
        is_valid, error_msg = await translation_service.validate_request(request)
        if not is_valid:
            raise HTTPException(status_code=400, detail=error_msg)

        # Perform translation
        result = await translation_service.translate(request)

        # Check if translation was successful
        if result.translated_text == request.text and result.confidence < 0.5:
            # Translation wasn't performed, return with warning
            logger.warning(f"Translation failed or was skipped for text: {request.text[:50]}...")
            return TranslationResponse(
                translated_text=request.text,
                source_language=request.source_language,
                target_language=request.target_language,
                translation_engine=TranslationEngine.OPENSOURCE,  # Use default engine
                confidence=0.0,
                is_machine_translated=False,
                cache_hit=result.cache_hit
            )

        return result

    except HTTPException:
        # Re-raise HTTP exceptions (like validation errors)
        raise
    except Exception as e:
        logger.error(f"Error in translation endpoint: {e}")
        # Return original text with warning instead of throwing error
        return TranslationResponse(
            translated_text=request.text,
            source_language=request.source_language,
            target_language=request.target_language,
            translation_engine=TranslationEngine.OPENSOURCE,
            confidence=0.0,
            is_machine_translated=False,
            cache_hit=False
        )


@router.get("/health")
async def translation_health():
    """Health check endpoint for the translation service"""
    return {"status": "healthy", "service": "translation"}


# Additional endpoints can be added here as needed
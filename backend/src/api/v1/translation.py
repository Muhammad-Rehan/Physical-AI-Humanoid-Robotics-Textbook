from fastapi import APIRouter, HTTPException
import logging
from typing import List

from ...models.translation import TranslationRequest, TranslationResponse
from ...services.translation_service import TranslationService

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize the translation service
translation_service = TranslationService(cache_ttl_seconds=3600)

@router.post("/translate", response_model=TranslationResponse)
async def translate_text(request: TranslationRequest):
    """
    Translate a batch of texts to the requested target language.
    Uses open-source models.
    """
    try:
        # Basic validation
        if not request.texts:
            raise HTTPException(status_code=400, detail="Input texts list cannot be empty.")
        for text in request.texts:
            if len(text) > 5000: # Limit length per string
                raise HTTPException(status_code=400, detail="One or more texts exceed the maximum length of 5000 characters.")

        # Perform batch translation
        translated_texts, engine, cache_hit = await translation_service.translate_batch(
            texts=request.texts,
            target_lang=request.target_language,
            source_lang=request.source_language
        )

        return TranslationResponse(
            translated_texts=translated_texts,
            source_language=request.source_language,
            target_language=request.target_language,
            translation_engine=engine,
            is_machine_translated=True,
            cache_hit=cache_hit # This will be an approximation for batches
        )

    except HTTPException as e:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in batch translation endpoint: {e}")
        # In case of failure, return original texts
        return TranslationResponse(
            translated_texts=request.texts,
            source_language=request.source_language,
            target_language=request.target_language,
            translation_engine="opensource",
            is_machine_translated=False,
            cache_hit=False
        )

@router.get("/health")
async def translation_health():
    """Health check endpoint for the translation service"""
    return {"status": "healthy", "service": "translation"}
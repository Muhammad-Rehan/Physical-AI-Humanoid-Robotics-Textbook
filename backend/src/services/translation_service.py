from abc import ABC, abstractmethod
from typing import Optional
from ..models.translation import (
    TranslationRequest,
    TranslationResponse,
    TranslationEngine,
    TranslationCacheEntry
)
import hashlib
import time
from datetime import datetime, timedelta


class ITranslator(ABC):
    """Abstract interface for translation engines"""

    @abstractmethod
    async def translate(self, text: str, target_lang: str, source_lang: str = "en") -> tuple[str, float]:
        """
        Translate text from source language to target language
        Returns: (translated_text, confidence_score)
        """
        pass

    @abstractmethod
    def is_language_supported(self, lang_code: str) -> bool:
        """Check if the language is supported by this translator"""
        pass


class TranslationService:
    """Main translation service orchestrating different translation engines"""

    def __init__(self, cache_ttl_seconds: int = 3600):
        self.argos_translator = None
        self.marianmt_translator = None
        self.cache = {}  # In-memory cache
        self.cache_ttl_seconds = cache_ttl_seconds
        self._initialize_translators()

    def _initialize_translators(self):
        """Initialize the translation engines"""
        try:
            from .open_source_translator import ArgosTranslator, MarianMTTranslator
            self.argos_translator = ArgosTranslator()
            self.marianmt_translator = MarianMTTranslator()
        except ImportError as e:
            print(f"Warning: Could not import translators: {e}")
            # Fallback translators that return original text
            from .fallback_translators import ArgosTranslator, MarianMTTranslator
            self.argos_translator = ArgosTranslator()
            self.marianmt_translator = MarianMTTranslator()

    def _generate_text_hash(self, text: str, source_lang: str, target_lang: str) -> str:
        """Generate a hash for the source text to use as cache key"""
        cache_key = f"{text}:{source_lang}:{target_lang}"
        return hashlib.sha256(cache_key.encode()).hexdigest()

    def _get_from_cache(self, text_hash: str) -> Optional[TranslationCacheEntry]:
        """Get translation from cache if it exists and hasn't expired"""
        if text_hash in self.cache:
            entry = self.cache[text_hash]
            if datetime.now() < entry.expires_at:
                # Update hit count
                entry.hit_count += 1
                return entry
            else:
                # Remove expired entry
                del self.cache[text_hash]
        return None

    def _add_to_cache(self, entry: TranslationCacheEntry):
        """Add translation to cache"""
        self.cache[entry.source_text_hash] = entry

    async def translate(
        self,
        request: TranslationRequest
    ) -> TranslationResponse:
        """
        Main translation method that orchestrates the translation process
        Uses Argos as default, falls back to MarianMT if needed
        """
        # Generate cache key
        text_hash = self._generate_text_hash(
            request.text,
            request.source_language,
            request.target_language
        )

        # Check cache first
        cached_entry = self._get_from_cache(text_hash)
        if cached_entry:
            return TranslationResponse(
                translated_text=cached_entry.translated_text,
                source_language=cached_entry.source_language,
                target_language=cached_entry.target_language,
                translation_engine=cached_entry.translation_engine,
                confidence=None,  # Cache doesn't store confidence
                is_machine_translated=True,
                cache_hit=True
            )

        # Validate language support
        if not self.argos_translator.is_language_supported(request.target_language):
            # Language not supported by Argos, try MarianMT
            if self.marianmt_translator.is_language_supported(request.target_language):
                translated_text, confidence = await self.marianmt_translator.translate(
                    request.text,
                    request.target_language,
                    request.source_language
                )
                engine = TranslationEngine.OPENSOURCE  # MarianMT is still open source
            else:
                # Neither translator supports the language, return original with warning
                return TranslationResponse(
                    translated_text=request.text,
                    source_language=request.source_language,
                    target_language=request.target_language,
                    translation_engine=TranslationEngine.OPENSOURCE,
                    confidence=0.0,
                    is_machine_translated=False,
                    cache_hit=False
                )
        else:
            # Try Argos first
            translated_text, confidence = await self.argos_translator.translate(
                request.text,
                request.target_language,
                request.source_language
            )

            # If Argos confidence is low, try MarianMT as fallback
            if confidence < 0.5:
                try:
                    marian_translated, marian_conf = await self.marianmt_translator.translate(
                        request.text,
                        request.target_language,
                        request.source_language
                    )
                    # Use the better translation
                    if marian_conf > confidence:
                        translated_text = marian_translated
                        confidence = marian_conf
                except Exception:
                    # If MarianMT fails, continue with Argos result
                    pass

            engine = TranslationEngine.OPENSOURCE  # Both are open source engines

        # Create cache entry
        cache_entry = TranslationCacheEntry(
            source_text_hash=text_hash,
            source_language=request.source_language,
            target_language=request.target_language,
            translated_text=translated_text,
            translation_engine=engine,
            created_at=datetime.now(),
            expires_at=datetime.now() + timedelta(seconds=self.cache_ttl_seconds),
            hit_count=1
        )

        # Add to cache
        self._add_to_cache(cache_entry)

        # Return response
        return TranslationResponse(
            translated_text=translated_text,
            source_language=request.source_language,
            target_language=request.target_language,
            translation_engine=engine,
            confidence=confidence,
            is_machine_translated=True,
            cache_hit=False
        )

    async def validate_request(self, request: TranslationRequest) -> tuple[bool, Optional[str]]:
        """Validate the translation request"""
        if not request.text or len(request.text.strip()) == 0:
            return False, "Text cannot be empty"

        if len(request.text) > 10000:
            return False, "Text exceeds maximum length of 10000 characters"

        if not request.target_language or len(request.target_language) != 2:
            return False, "Target language must be a valid 2-letter language code"

        # Check if context is valid if provided
        if request.context and request.context.value not in ["page_content", "selected_text", "chatbot_response"]:
            return False, "Invalid context provided"

        return True, None
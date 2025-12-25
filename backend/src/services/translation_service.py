from abc import ABC, abstractmethod
from typing import List, Tuple
from ..models.translation import TranslationEngine, TranslationCacheEntry
import hashlib
from datetime import datetime, timedelta
import logging

logger = logging.getLogger(__name__)

class ITranslator(ABC):
    """Abstract interface for translation engines"""

    @abstractmethod
    async def translate_batch(self, texts: List[str], target_lang: str, source_lang: str) -> List[str]:
        """
        Translate a batch of texts from source language to target language.
        Returns a list of translated texts in the same order as the input.
        """
        pass

    @abstractmethod
    def is_language_supported(self, lang_code: str) -> bool:
        """Check if the language is supported by this translator"""
        pass


class TranslationService:
    """Main translation service orchestrating different translation engines"""

    def __init__(self, cache_ttl_seconds: int = 3600):
        self.translator: ITranslator = None
        self.cache = {}  # In-memory cache
        self.cache_ttl_seconds = cache_ttl_seconds
        self._initialize_translators()

    def _initialize_translators(self):
        """Initialize the translation engines"""
        try:
            from .openrouter_translator import OpenRouterTranslator
            self.translator = OpenRouterTranslator()
            logger.info("Using OpenRouter for translation.")
        except ImportError as e:
            logger.error(f"Could not import OpenRouterTranslator: {e}")
            # As a fallback, we could implement a dummy translator here if needed

    def _generate_text_hash(self, text: str, source_lang: str, target_lang: str) -> str:
        """Generate a hash for the source text to use as cache key"""
        cache_key = f"{text}:{source_lang}:{target_lang}"
        return hashlib.sha256(cache_key.encode()).hexdigest()

    def _get_from_cache(self, text_hash: str) -> str | None:
        """Get translation from cache if it exists and hasn't expired"""
        if text_hash in self.cache:
            entry = self.cache[text_hash]
            if datetime.now() < entry.expires_at:
                entry.hit_count += 1
                return entry.translated_text
            else:
                del self.cache[text_hash]
        return None

    def _add_to_cache(self, text_hash: str, translated_text: str, source_lang: str, target_lang: str):
        """Add translation to cache"""
        entry = TranslationCacheEntry(
            source_text_hash=text_hash,
            source_language=source_lang,
            target_language=target_lang,
            translated_text=translated_text,
            translation_engine=TranslationEngine.LLM, # Using LLM via OpenRouter
            created_at=datetime.now(),
            expires_at=datetime.now() + timedelta(seconds=self.cache_ttl_seconds),
            hit_count=1
        )
        self.cache[entry.source_text_hash] = entry

    async def translate_batch(
        self,
        texts: List[str],
        target_lang: str,
        source_lang: str
    ) -> Tuple[List[str], str, bool]:
        """
        Translates a batch of texts, using caching and the OpenRouter translation engine.
        """
        final_translations = [""] * len(texts)
        to_translate_indices = []
        to_translate_texts = []
        cache_hit = False

        # 1. Check cache for each text
        for i, text in enumerate(texts):
            if not text.strip():
                final_translations[i] = text
                continue

            text_hash = self._generate_text_hash(text, source_lang, target_lang)
            cached_translation = self._get_from_cache(text_hash)

            if cached_translation:
                final_translations[i] = cached_translation
                cache_hit = True
            else:
                to_translate_indices.append(i)
                to_translate_texts.append(text)

        # 2. Translate texts that were not in cache
        if to_translate_texts:
            engine = TranslationEngine.LLM
            translated_batch = await self.translator.translate_batch(
                to_translate_texts,
                target_lang,
                source_lang
            )

            # 3. Populate final results and update cache
            for i, translated_text in enumerate(translated_batch):
                original_index = to_translate_indices[i]
                original_text = to_translate_texts[i]

                final_translations[original_index] = translated_text

                # Update cache
                text_hash = self._generate_text_hash(original_text, source_lang, target_lang)
                self._add_to_cache(text_hash, translated_text, source_lang, target_lang)

        return final_translations, TranslationEngine.LLM, cache_hit
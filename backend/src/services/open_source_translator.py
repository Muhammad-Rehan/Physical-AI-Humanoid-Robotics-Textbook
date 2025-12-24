from .translation_service import ITranslator
from typing import Tuple, Optional
import logging

logger = logging.getLogger(__name__)


class ArgosTranslator(ITranslator):
    """Implementation of ITranslator using Argos Translate"""

    def __init__(self):
        self._available_languages = set()
        self._load_supported_languages()

    def _load_supported_languages(self):
        """Load supported languages from Argos Translate"""
        try:
            from argostranslate import translate
            # Update packages to get latest language support
            translate.update_available_packages()

            # Get available packages and extract language codes
            packages = translate.get_available_packages()
            for pkg in packages:
                self._available_languages.add(pkg.from_code)
                self._available_languages.add(pkg.to_code)
        except ImportError:
            logger.warning("Argos Translate not available, using fallback")
            # Default to common languages if argos is not installed
            self._available_languages = {'en', 'es', 'fr', 'de', 'it', 'pt', 'ru', 'zh', 'ja', 'ar'}
        except Exception as e:
            logger.error(f"Error loading Argos languages: {e}")
            # Default to common languages
            self._available_languages = {'en', 'es', 'fr', 'de', 'it', 'pt', 'ru', 'zh', 'ja', 'ar'}

    async def translate(self, text: str, target_lang: str, source_lang: str = "en") -> Tuple[str, float]:
        """Translate text using Argos Translate"""
        try:
            from argostranslate import translate as argos_translate
            from argostranslate.translate import get_installed_translators

            # Check if the language pair is installed
            installed_translators = get_installed_translators()
            translator_found = False

            for translator in installed_translators:
                if (translator.from_code == source_lang and translator.to_code == target_lang):
                    translator_found = True
                    break

            if not translator_found:
                # If direct translation is not available, try to find a path through English
                if source_lang != 'en' and target_lang != 'en':
                    # Try source -> en -> target
                    intermediate_result, _ = await self.translate(text, 'en', source_lang)
                    final_result, _ = await self.translate(intermediate_result, target_lang, 'en')
                    return final_result, 0.7  # Lower confidence for multi-step translation
                else:
                    # Try to install the required package
                    available_packages = argos_translate.get_available_packages()
                    for pkg in available_packages:
                        if pkg.from_code == source_lang and pkg.to_code == target_lang:
                            try:
                                pkg.install()
                                break
                            except Exception as e:
                                logger.warning(f"Could not install package {pkg}: {e}")

            # Perform the translation
            installed_translators = get_installed_translators()
            for translator in installed_translators:
                if translator.from_code == source_lang and translator.to_code == target_lang:
                    translated_text = translator.translate(text)
                    # Estimate confidence based on success (this is a simple approach)
                    confidence = 0.9 if len(translated_text) > 0 else 0.3
                    return translated_text, confidence

            # If no direct translator found, return original text with low confidence
            return text, 0.1

        except ImportError:
            logger.warning("Argos Translate not installed, returning original text")
            return text, 0.1
        except Exception as e:
            logger.error(f"Error in Argos translation: {e}")
            return text, 0.1

    def is_language_supported(self, lang_code: str) -> bool:
        """Check if the language is supported"""
        return lang_code in self._available_languages


class MarianMTTranslator(ITranslator):
    """Implementation of ITranslator using MarianMT models"""

    def __init__(self):
        self._available_models = {}
        self._load_available_models()

    def _load_available_models(self):
        """Load available MarianMT models"""
        # Define common language pairs
        self._available_models = {
            ('en', 'es'), ('es', 'en'),
            ('en', 'fr'), ('fr', 'en'),
            ('en', 'de'), ('de', 'en'),
            ('en', 'it'), ('it', 'en'),
            ('en', 'pt'), ('pt', 'en'),
            ('en', 'ru'), ('ru', 'en'),
            ('en', 'zh'), ('zh', 'en'),
            ('en', 'ja'), ('ja', 'en'),
            ('en', 'ar'), ('ar', 'en'),
        }

    async def translate(self, text: str, target_lang: str, source_lang: str = "en") -> Tuple[str, float]:
        """Translate text using MarianMT models"""
        try:
            from transformers import pipeline, AutoTokenizer, AutoModelForSeq2SeqLM
            import torch

            # Check if this language pair is supported
            if (source_lang, target_lang) not in self._available_models:
                # Try to construct model name based on language codes
                model_name = f"Helsinki-NLP/opus-mt-{source_lang}-{target_lang}"
            else:
                model_name = f"Helsinki-NLP/opus-mt-{source_lang}-{target_lang}"

            try:
                # Load tokenizer and model
                tokenizer = AutoTokenizer.from_pretrained(model_name)
                model = AutoModelForSeq2SeqLM.from_pretrained(model_name)

                # Tokenize input
                inputs = tokenizer(text, return_tensors="pt", padding=True, truncation=True, max_length=512)

                # Generate translation
                with torch.no_grad():
                    outputs = model.generate(**inputs, max_length=512, num_beams=5, early_stopping=True)

                # Decode output
                translated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)

                # Return with medium confidence (MarianMT quality)
                return translated_text, 0.8
            except Exception as e:
                # Try alternative model naming convention
                alt_model_name = f"Helsinki-NLP/opus-mt-{target_lang}-{source_lang}"
                try:
                    tokenizer = AutoTokenizer.from_pretrained(alt_model_name)
                    model = AutoModelForSeq2SeqLM.from_pretrained(alt_model_name)

                    inputs = tokenizer(text, return_tensors="pt", padding=True, truncation=True, max_length=512)

                    with torch.no_grad():
                        outputs = model.generate(**inputs, max_length=512, num_beams=5, early_stopping=True)

                    translated_text = tokenizer.decode(outputs[0], skip_special_tokens=True)
                    return translated_text, 0.8
                except Exception as e2:
                    logger.error(f"Error in MarianMT translation: {e}, {e2}")
                    return text, 0.1

        except ImportError:
            logger.warning("Transformers library not installed, returning original text")
            return text, 0.1
        except Exception as e:
            logger.error(f"Error in MarianMT translation: {e}")
            return text, 0.1

    def is_language_supported(self, lang_code: str) -> bool:
        """Check if the language is supported by checking if any model includes this language"""
        return any(lang_code in model_pair for model_pair in self._available_models)


# Fallback implementations for when required libraries are not available
class FallbackArgosTranslator(ITranslator):
    """Fallback implementation when Argos Translate is not available"""

    def __init__(self):
        # Common language codes for fallback
        self._supported_languages = {'en', 'es', 'fr', 'de', 'it', 'pt', 'ru', 'zh', 'ja', 'ar', 'ko', 'hi'}

    async def translate(self, text: str, target_lang: str, source_lang: str = "en") -> Tuple[str, float]:
        """Fallback translation - just return original text with low confidence"""
        return text, 0.1

    def is_language_supported(self, lang_code: str) -> bool:
        """Check if language is in our supported list"""
        return lang_code in self._supported_languages


class FallbackMarianMTTranslator(ITranslator):
    """Fallback implementation when MarianMT is not available"""

    def __init__(self):
        # Common language codes for fallback
        self._supported_languages = {'en', 'es', 'fr', 'de', 'it', 'pt', 'ru', 'zh', 'ja', 'ar', 'ko', 'hi'}

    async def translate(self, text: str, target_lang: str, source_lang: str = "en") -> Tuple[str, float]:
        """Fallback translation - just return original text with low confidence"""
        return text, 0.1

    def is_language_supported(self, lang_code: str) -> bool:
        """Check if language is in our supported list"""
        return lang_code in self._supported_languages
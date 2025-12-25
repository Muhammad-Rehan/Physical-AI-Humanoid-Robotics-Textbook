from .translation_service import ITranslator
from typing import List
import os
import openai # Use the openai client
import asyncio
import logging

logger = logging.getLogger(__name__)

class OpenRouterTranslator(ITranslator):
    """Implementation of ITranslator using the OpenRouter API via OpenAI-compatible client."""

    def __init__(self):
        self.api_key = os.getenv("OPENROUTER_TRANSLATION_API_KEY")
        if not self.api_key:
            logger.warning("OPENROUTER_TRANSLATION_API_KEY not found in environment variables. Translation will fail.")
        
        # Create a dedicated AsyncOpenAI client instance for OpenRouter translation
        self.client = openai.AsyncOpenAI(
            api_key=self.api_key,
            base_url="https://openrouter.ai/api/v1"
        )

    async def _translate_single_text(self, text: str, target_lang: str, source_lang: str) -> str:
        """Translates a single piece of text using OpenRouter."""
        if not self.api_key:
            return text

        try:
            model = "xiaomi/mimo-v2-flash:free" # Correct model ID provided by the user

            response = await self.client.chat.completions.create(
                model=model,
                messages=[
                    {
                        "role": "system",
                        "content": f"You are a translation expert. Translate the user's text from {source_lang} to {target_lang}. Respond only with the translated text, nothing else."
                    },
                    {
                        "role": "user",
                        "content": text
                    }
                ],
                temperature=0.3,
                max_tokens=2048,
                timeout=30.0, # 30-second timeout for the API call
            )
            if response.choices:
                return response.choices[0].message.content.strip()
            return text
        except Exception as e:
            logger.error(f"Error during OpenRouter translation for text '{text[:40]}...': {e}")
            return text

    async def translate_batch(self, texts: List[str], target_lang: str, source_lang: str = "en") -> List[str]:
        """Translate a batch of texts using OpenRouter, running translations in parallel."""
        try:
            tasks = [self._translate_single_text(text, target_lang, source_lang) for text in texts]
            # Wait for all tasks to complete, with an overall timeout for the batch
            translated_texts = await asyncio.wait_for(asyncio.gather(*tasks), timeout=60.0)
            return translated_texts
        except asyncio.TimeoutError:
            logger.error("Batch translation timed out after 60 seconds.")
            return texts # Return original texts on timeout
        except Exception as e:
            logger.error(f"An unexpected error occurred during batch translation: {e}")
            return texts

    def is_language_supported(self, lang_code: str) -> bool:
        """
        LLMs support a vast number of languages. We assume true.
        """
        return True

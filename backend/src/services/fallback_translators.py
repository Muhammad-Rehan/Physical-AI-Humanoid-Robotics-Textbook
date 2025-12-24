from .open_source_translator import FallbackArgosTranslator, FallbackMarianMTTranslator

# Expose the fallback classes for import
ArgosTranslator = FallbackArgosTranslator
MarianMTTranslator = FallbackMarianMTTranslator
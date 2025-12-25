from pydantic import BaseModel, Field
from typing import Optional, List
from enum import Enum
from datetime import datetime


class TranslationContext(str, Enum):
    """Enumeration of possible translation contexts"""
    PAGE_CONTENT = "page_content"
    SELECTED_TEXT = "selected_text"
    CHATBOT_RESPONSE = "chatbot_response"


class TranslationEngine(str, Enum):
    """Enumeration of translation engines"""
    OPENSOURCE = "opensource"
    LLM = "llm"


class TranslationRequest(BaseModel):
    """Model for translation requests"""
    texts: List[str] = Field(
        ...,
        description="A list of source texts to be translated",
        max_length=1000
    )
    target_language: str = Field(
        ...,
        description="The ISO language code for the target language",
        pattern=r"^[a-z]{2}$",
        example="es"
    )
    source_language: Optional[str] = Field(
        "en",
        description="The ISO language code for the source language",
        pattern=r"^[a-z]{2}$",
        example="en"
    )
    context: Optional[TranslationContext] = Field(
        TranslationContext.PAGE_CONTENT,
        description="Context information for the translation"
    )


class TranslationResponse(BaseModel):
    """Model for translation responses"""
    translated_texts: List[str] = Field(
        ...,
        description="The list of translated texts"
    )
    source_language: str = Field(
        ...,
        description="The detected source language",
        pattern=r"^[a-z]{2}$"
    )
    target_language: str = Field(
        ...,
        description="The requested target language",
        pattern=r"^[a-z]{2}$"
    )
    translation_engine: TranslationEngine = Field(
        ...,
        description="Which engine was used for translation"
    )
    is_machine_translated: bool = Field(
        True,
        description="Flag indicating this is machine translated content"
    )
    cache_hit: bool = Field(
        False,
        description="Whether the translation was retrieved from cache"
    )


class ErrorResponse(BaseModel):
    """Model for error responses"""
    error: str = Field(
        ...,
        description="Error code"
    )
    message: str = Field(
        ...,
        description="Error description"
    )
    details: Optional[dict] = Field(
        None,
        description="Additional error details"
    )


class TranslationCacheEntry(BaseModel):
    """Model for translation cache entries"""
    source_text_hash: str = Field(
        ...,
        description="Hash of the source text for quick lookup"
    )
    source_language: str = Field(
        ...,
        description="Source language code",
        pattern=r"^[a-z]{2}$"
    )
    target_language: str = Field(
        ...,
        description="Target language code",
        pattern=r"^[a-z]{2}$"
    )
    translated_text: str = Field(
        ...,
        description="Cached translated text"
    )
    translation_engine: TranslationEngine = Field(
        ...,
        description="Engine used for translation"
    )
    created_at: datetime = Field(
        ...,
        description="Timestamp when entry was created"
    )
    expires_at: datetime = Field(
        ...,
        description="Timestamp when entry expires"
    )
    hit_count: int = Field(
        0,
        description="Number of times this translation was used",
        ge=0
    )


class LanguagePreference(BaseModel):
    """Model for user language preferences"""
    user_id: Optional[str] = Field(
        None,
        description="The user identifier (if authenticated)"
    )
    selected_language: str = Field(
        ...,
        description="The user's currently selected language",
        pattern=r"^[a-z]{2}$",
        example="es"
    )
    supported_languages: List[str] = Field(
        default_factory=list,
        description="List of supported language codes"
    )
    last_updated: Optional[datetime] = Field(
        None,
        description="Timestamp of last preference update"
    )
import React, { useState, useEffect, useCallback } from 'react';
import TranslationIndicator from '../TranslationIndicator';
import apiService from '../../services/api';
import './TranslationWrapper.css';

// Helper function to recursively extract text from children
const extractTextFromChildren = (children) => {
  if (typeof children === 'string') {
    return children;
  }
  if (Array.isArray(children)) {
    return children.map(extractTextFromChildren).join('');
  }
  if (React.isValidElement(children) && children.props.children) {
    return extractTextFromChildren(children.props.children);
  }
  return '';
};


const TranslationWrapper = ({
  children,
  content,
  context = 'page_content',
  sourceLanguage = 'en',
  targetLanguage,
  showIndicator = true
}) => {
  const [translatedContent, setTranslatedContent] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [originalContent, setOriginalContent] = useState(content || children);
  const [isTranslated, setIsTranslated] = useState(false);
  const [translationInfo, setTranslationInfo] = useState(null);

  // Get user's preferred language from localStorage or default
  useEffect(() => {
    const savedLanguage = localStorage.getItem('preferredLanguage') || 'en';
    if (targetLanguage !== savedLanguage) {
      // Note: We're not setting state here as it would create a circular dependency
      // The parent component should handle this
    }
  }, []);

  // Translate content when target language changes
  useEffect(() => {
    if (targetLanguage && targetLanguage !== sourceLanguage && (content || children)) {
      translateContent();
    }
  }, [content, children, targetLanguage, sourceLanguage, context]);

  const translateContent = useCallback(async () => {
    const textToTranslate = extractTextFromChildren(content || children);
    if (!targetLanguage || targetLanguage === sourceLanguage || !textToTranslate) {
      setTranslatedContent(null);
      setIsTranslated(false);
      setTranslationInfo(null);
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Call the actual translation API
      const response = await apiService.translateText(
        textToTranslate,
        targetLanguage,
        sourceLanguage,
        context
      );

      if (response.translated_text && response.translated_text !== textToTranslate) {
        setTranslatedContent(response.translated_text);
setIsTranslated(true);
        setTranslationInfo(response);
      } else {
        setTranslatedContent(textToTranslate); // Fallback to original content
        setIsTranslated(false);
        setTranslationInfo(null);
      }
    } catch (err) {
      console.error('Translation error:', err);
      setError(err.message);
      setTranslatedContent(textToTranslate); // Fallback to original content
      setIsTranslated(false);
      setTranslationInfo(null);
    } finally {
      setIsLoading(false);
    }
  }, [content, children, targetLanguage, sourceLanguage, context]);

  // Function to revert to original content
  const revertToOriginal = () => {
    setTranslatedContent(null);
    setIsTranslated(false);
    setTranslationInfo(null);
  };

  // Determine what content to display
  const displayContent = translatedContent !== null ? translatedContent : (content || children);

  return (
    <div className="translation-wrapper">
      {isLoading && <div className="translation-loading">Translating...</div>}

      {error && (
        <div className="translation-error">
          Translation error: {error}. Showing original content.
        </div>
      )}

      {isTranslated && showIndicator && (
        <div className="machine-translation-indicator">
          <span className="indicator-text">🌐 Machine Translated</span>
          {translationInfo?.confidence && (
            <span className="confidence-score">Confidence: {(translationInfo.confidence * 100).toFixed(0)}%</span>
          )}
          <button className="revert-button" onClick={revertToOriginal} title="Show original text">
            Show Original
          </button>
        </div>
      )}

      {showIndicator && targetLanguage && targetLanguage !== sourceLanguage && !isLoading && isTranslated && (
        <TranslationIndicator
          showIndicator={true}
          sourceLanguage={sourceLanguage}
          targetLanguage={targetLanguage}
        />
      )}

      <div className="translation-content">
        {displayContent}
      </div>
    </div>
  );
};

export default TranslationWrapper;
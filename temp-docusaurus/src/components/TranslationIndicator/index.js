import React from 'react';
import './TranslationIndicator.css';

const TranslationIndicator = ({ showIndicator = true, sourceLanguage = 'en', targetLanguage = 'es' }) => {
  if (!showIndicator) {
    return null;
  }

  return (
    <div className="translation-indicator">
      <div className="indicator-content">
        <span className="indicator-icon">🌐</span>
        <span className="indicator-text">
          Content translated from {getLanguageName(sourceLanguage)} to {getLanguageName(targetLanguage)}
        </span>
        <button className="indicator-info-btn" title="Translation information">
          ⓘ
        </button>
      </div>
    </div>
  );
};

const getLanguageName = (languageCode) => {
  const languageNames = {
    'en': 'English',
    'es': 'Spanish',
    'fr': 'French',
    'de': 'German',
    'it': 'Italian',
    'pt': 'Portuguese',
    'ru': 'Russian',
    'zh': 'Chinese',
    'ja': 'Japanese',
    'ar': 'Arabic',
    'hi': 'Hindi',
    'ur': 'Urdu'
  };
  return languageNames[languageCode] || languageCode;
};

export default TranslationIndicator;
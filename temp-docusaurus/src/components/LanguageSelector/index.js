import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';
import './LanguageSelector.css';

const LANGUAGES = [
  { code: 'en', name: 'English' },
  { code: 'es', name: 'Spanish' },
  { code: 'fr', name: 'French' },
  { code: 'de', name: 'German' },
  { code: 'it', name: 'Italian' },
  { code: 'pt', name: 'Portuguese' },
  { code: 'ru', name: 'Russian' },
  { code: 'zh', name: 'Chinese' },
  { code: 'ja', name: 'Japanese' },
  { code: 'ar', name: 'Arabic' },
  { code: 'hi', name: 'Hindi' },
  { code: 'ur', name: 'Urdu' }
];

const LanguageSelectorImpl = ({ onLanguageChange, currentLanguage }) => {
  const [selectedLanguage, setSelectedLanguage] = useState(() => {
    // Get saved language from localStorage or use currentLanguage prop or default to 'en'
    return localStorage.getItem('preferredLanguage') || currentLanguage || 'en';
  });
  const [isOpen, setIsOpen] = useState(false);

  useEffect(() => {
    // Update selected language if currentLanguage prop changes
    if (currentLanguage && currentLanguage !== selectedLanguage) {
      setSelectedLanguage(currentLanguage);
    }
  }, [currentLanguage, selectedLanguage]);

  const handleLanguageSelect = (languageCode) => {
    setSelectedLanguage(languageCode);
    setIsOpen(false);
    if (onLanguageChange) {
      onLanguageChange(languageCode);
    }
    // Store user preference in localStorage
    localStorage.setItem('preferredLanguage', languageCode);

    // Dispatch a custom event to notify other components of language change
    window.dispatchEvent(new CustomEvent('languageChanged', { detail: { language: languageCode } }));
  };

  const toggleDropdown = () => {
    setIsOpen(!isOpen);
  };

  const currentLanguageName = LANGUAGES.find(lang => lang.code === selectedLanguage)?.name || 'English';

  return (
    <div className="language-selector">
      <button
        className="language-selector-button"
        onClick={toggleDropdown}
        aria-label="Select language"
        title={`Current language: ${currentLanguageName}`}
      >
        <span className="language-flag">{getFlagEmoji(selectedLanguage)}</span>
        <span className="language-name">{currentLanguageName}</span>
        <span className="dropdown-arrow">{isOpen ? '▲' : '▼'}</span>
      </button>

      {isOpen && (
        <div className="language-dropdown">
          <div className="language-search">
            <input
              type="text"
              placeholder="Search languages..."
              className="language-search-input"
            />
          </div>
          <ul className="language-list">
            {LANGUAGES.map((language) => (
              <li
                key={language.code}
                className={`language-option ${language.code === selectedLanguage ? 'selected' : ''}`}
                onClick={() => handleLanguageSelect(language.code)}
              >
                <span className="language-flag">{getFlagEmoji(language.code)}</span>
                <span className="language-name">{language.name}</span>
                {language.code === selectedLanguage && (
                  <span className="selected-check">✓</span>
                )}
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
};

// Helper function to get flag emoji for language
const getFlagEmoji = (languageCode) => {
  const flags = {
    'en': '🇬🇧',
    'es': '🇪🇸',
    'fr': '🇫🇷',
    'de': '🇩🇪',
    'it': '🇮🇹',
    'pt': '🇵🇹',
    'ru': '🇷🇺',
    'zh': '🇨🇳',
    'ja': '🇯🇵',
    'ar': '🇸🇦',
    'hi': '🇮🇳',
    'ur': '🇵🇰'
  };
  return flags[languageCode] || '🌐';
};

const LanguageSelector = (props) => {
  return (
    <BrowserOnly>
      {() => <LanguageSelectorImpl {...props} />}
    </BrowserOnly>
  );
};

export default LanguageSelector;
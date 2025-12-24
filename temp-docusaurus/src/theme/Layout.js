import React, { useState, useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';
import RagChatbot from '../components/RagChatbot';
import LanguageSelector from '../components/LanguageSelector';
import TranslationWrapper from '../components/TranslationWrapper';

export default function Layout(props) {
  const [targetLanguage, setTargetLanguage] = useState('en');

  useEffect(() => {
    const savedLanguage = localStorage.getItem('preferredLanguage') || 'en';
    setTargetLanguage(savedLanguage);

    const handleLanguageChange = (event) => {
      setTargetLanguage(event.detail.language);
    };

    window.addEventListener('languageChanged', handleLanguageChange);

    return () => {
      window.removeEventListener('languageChanged', handleLanguageChange);
    };
  }, []);

  const handleLanguageChange = (languageCode) => {
    setTargetLanguage(languageCode);
    localStorage.setItem('preferredLanguage', languageCode);
  };

  return (
    <OriginalLayout {...props}>
      <TranslationWrapper targetLanguage={targetLanguage}>
        {props.children}
      </TranslationWrapper>
      <div style={{
        position: 'fixed',
        top: '18px',
        right: '130px',
        zIndex: 1000,
        display: 'flex',
        alignItems: 'center'
      }}>
        <LanguageSelector
          onLanguageChange={handleLanguageChange}
          currentLanguage={targetLanguage}
        />
      </div>
      <RagChatbot />
    </OriginalLayout>
  );
}
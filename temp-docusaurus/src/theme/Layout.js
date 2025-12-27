import React, { useState, useEffect, useRef } from 'react';
import OriginalLayout from '@theme-original/Layout';
import RagChatbot from '../components/RagChatbot';
import LanguageSelector from '../components/LanguageSelector';
import TranslationWrapper from '../components/TranslationWrapper';

export default function Layout(props) {
  const [targetLanguage, setTargetLanguage] = useState('en');
  const [selectedText, setSelectedText] = useState('');
  const [showTooltip, setShowTooltip] = useState(false);
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });
  const [chatbotInputValue, setChatbotInputValue] = useState('');
  const tooltipRef = useRef(null);

  // Show tooltip on text selection
  useEffect(() => {
    const handleSelection = () => {
      const text = window.getSelection().toString();
      if (text) {
        setSelectedText(text);
        setShowTooltip(true);
        const selection = window.getSelection();
        if (selection.rangeCount > 0) {
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();
          setTooltipPosition({
            x: rect.left + window.scrollX + (rect.width / 2) - 50, // Center the tooltip
            y: rect.top + window.scrollY - 40, // Position above selection
          });
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  // Hide tooltip on click outside or scroll
  useEffect(() => {
    const handleClickOutside = (event) => {
      // Hide if tooltip is shown and click is outside of it
      if (tooltipRef.current && !tooltipRef.current.contains(event.target)) {
        setShowTooltip(false);
      }
    };

    const handleScroll = () => {
        setShowTooltip(false);
    };

    document.addEventListener('mousedown', handleClickOutside);
    window.addEventListener('scroll', handleScroll, true); // Use capture phase for scroll

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
      window.removeEventListener('scroll', handleScroll, true);
    };
  }, []);


  const handleLanguageChange = (languageCode) => {
    setTargetLanguage(languageCode);
    localStorage.setItem('preferredLanguage', languageCode);
  };

  const handleTooltipClick = () => {
    setChatbotInputValue(selectedText);
    setShowTooltip(false); // Hide tooltip after click
  };

  const clearChatbotInputValue = () => {
    setChatbotInputValue('');
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
      <RagChatbot
        initialValue={chatbotInputValue}
        selectedText={selectedText}
        clearInitialValue={clearChatbotInputValue}
      />
      {showTooltip && selectedText && (
        <div
          ref={tooltipRef}
          className="selection-tooltip"
          style={{
            position: 'absolute',
            left: tooltipPosition.x,
            top: tooltipPosition.y,
            background: 'black',
            color: 'white',
            padding: '5px 10px',
            borderRadius: '5px',
            zIndex: 2000,
            cursor: 'pointer',
          }}
          onClick={handleTooltipClick}
        >
          Ask from Bot
        </div>
      )}
    </OriginalLayout>
  );
}
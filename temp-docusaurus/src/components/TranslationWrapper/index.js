import React, { useRef, useEffect, useState } from 'react';
import apiService from '../../services/api';
import './TranslationWrapper.css'; // Import the CSS file

const TranslationWrapper = ({ children, targetLanguage, sourceLanguage = 'en' }) => {
  const contentRef = useRef(null);
  const [isLoading, setIsLoading] = useState(false);

  useEffect(() => {
    if (!targetLanguage || !contentRef.current) {
      return;
    }

    if (targetLanguage === sourceLanguage) {
      revertToOriginal();
    } else {
      translateContent();
    }
  }, [targetLanguage]);

  const revertToOriginal = () => {
    const allElements = contentRef.current.querySelectorAll('[data-original-text]');
    allElements.forEach(element => {
      // This is a simplified revert. A more robust solution might need to
      // handle cases where multiple text nodes are inside one element.
      // For now, we restore the first child text node.
      const textNode = Array.from(element.childNodes).find(node => node.nodeType === Node.TEXT_NODE);
      if (textNode) {
        textNode.nodeValue = element.getAttribute('data-original-text');
      }
    });
  };

  const translateContent = async () => {
    setIsLoading(true);
    const textNodes = [];
    const textsToTranslate = [];

    // 1. Walk the DOM to find all text nodes
    const walk = (node) => {
      if (node.nodeType === 3) { // Node.TEXT_NODE
        const text = node.nodeValue.trim();
        if (text && text.length > 1) { // Ignore empty/whitespace-only text
          textNodes.push(node);
          textsToTranslate.push(text);
          // Store original text if not already stored
          if (!node.parentNode.hasAttribute('data-original-text')) {
            node.parentNode.setAttribute('data-original-text', text);
          }
        }
      } else if (node.nodeType === 1 && node.nodeName !== 'SCRIPT' && node.nodeName !== 'STYLE') { // Node.ELEMENT_NODE
        for (let i = 0; i < node.childNodes.length; i++) {
          walk(node.childNodes[i]);
        }
      }
    };

    walk(contentRef.current);
    

    if (textsToTranslate.length === 0) {
      setIsLoading(false);
      return;
    }

    try {
      // 2. Call the batch translation API
      const response = await apiService.translateText(textsToTranslate, targetLanguage, sourceLanguage);

      if (response.translated_texts && response.translated_texts.length === textsToTranslate.length) {
        // 3. Replace text nodes with translated content
        for (let i = 0; i < textNodes.length; i++) {
          textNodes[i].nodeValue = response.translated_texts[i];
        }
      }
    } catch (error) {
      console.error("[TranslationWrapper] Translation failed:", error);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="translation-wrapper">
      {isLoading && (
        <div className="translation-loading-overlay">
          <div className="spinner"></div>
          <div className="loading-text">Translating...</div>
        </div>
      )}
      <div ref={contentRef} className={isLoading ? 'translation-content-loading' : ''}>
        {children}
      </div>
    </div>
  );
};

export default TranslationWrapper;
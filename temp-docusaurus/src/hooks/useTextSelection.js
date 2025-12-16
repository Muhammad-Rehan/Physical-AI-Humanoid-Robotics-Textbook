import { useState, useEffect } from 'react';

const useTextSelection = () => {
  const [selectedText, setSelectedText] = useState('');

  useEffect(() => {
    const handleSelection = () => {
      const text = window.getSelection().toString().trim();
      setSelectedText(text);
    };

    // Add event listeners
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection); // For keyboard selection

    // Cleanup event listeners on unmount
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, []);

  const clearSelection = () => {
    setSelectedText('');
    window.getSelection().removeAllRanges();
  };

  return {
    selectedText,
    clearSelection
  };
};

export default useTextSelection;
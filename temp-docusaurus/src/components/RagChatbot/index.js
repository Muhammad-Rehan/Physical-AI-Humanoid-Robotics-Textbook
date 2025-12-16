import React, { useState, useEffect, useRef } from 'react';
import './styles.css';

const RagChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  // Function to get selected text from the page
  const getSelectedText = () => {
    const text = window.getSelection().toString().trim();
    return text;
  };

  // Handle text selection on the page
  useEffect(() => {
    const handleSelection = () => {
      const text = getSelectedText();
      if (text) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Auto-scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Prepare the request payload
      const requestBody = {
        question: inputValue,
        selected_text: selectedText || null
      };

      // Send request to backend
      const response = await fetch('http://localhost:8000/api/v1/chat/ask', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody)
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        sources: data.sources,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'bot',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <div className="rag-chatbot">
      {/* Chatbot toggle button */}
      <button
        className={`chatbot-toggle ${isOpen ? 'open' : ''}`}
        onClick={toggleChat}
        aria-label={isOpen ? "Close chat" : "Open chat"}
      >
        {isOpen ? '×' : '?'}
      </button>

      {/* Chatbot window */}
      {isOpen && (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>Textbook Assistant</h3>
            {selectedText && (
              <div className="selected-text-indicator">
                <span title={selectedText}>
                  {selectedText.length > 50 ? selectedText.substring(0, 50) + '...' : selectedText}
                </span>
              </div>
            )}
          </div>

          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your textbook assistant.</p>
                <p>You can ask me questions about the Physical AI & Humanoid Robotics content.</p>
                {selectedText && (
                  <p><em>Selected text mode: I'll answer based only on the highlighted text.</em></p>
                )}
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
                >
                  <div className="message-content">
                    {message.text}
                  </div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="message-sources">
                      <small>Sources: {message.sources.slice(0, 2).join(', ')}</small>
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div className="message bot-message">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chatbot-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={selectedText
                ? "Ask about the selected text..."
                : "Ask a question about the textbook content..."}
              rows="2"
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={!inputValue.trim() || isLoading}
              className="send-button"
            >
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default RagChatbot;
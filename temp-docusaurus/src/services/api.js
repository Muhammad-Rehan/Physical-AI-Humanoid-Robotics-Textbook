// API service for communicating with the RAG backend
// Using window global to avoid process.env issues in Docusaurus
const API_BASE_URL = 'https://muhammad1r-rag-chatbot.hf.space/api/v1';

class ApiService {
  constructor() {
    this.baseUrl = API_BASE_URL;
  }

  // Ask a question to the RAG system
  async askQuestion(question, selectedText = null) {
    try {
      const response = await fetch(`${this.baseUrl}/chat/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question,
          selected_text: selectedText || null
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error in askQuestion:', error);
      throw error;
    }
  }

  // Translate a batch of texts using the translation API
  async translateText(texts, targetLanguage, sourceLanguage = 'en', context = 'page_content') {
    try {
      const response = await fetch(`${this.baseUrl}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          texts,
          target_language: targetLanguage,
          source_language: sourceLanguage,
          context
        })
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || `HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error in translateText:', error);
      throw error;
    }
  }

  // Health check for the API
  async healthCheck() {
    try {
      const response = await fetch(`${this.baseUrl}/chat/health`);
      return await response.json();
    } catch (error) {
      console.error('Error in healthCheck:', error);
      throw error;
    }
  }

  // Health check for the translation API
  async translationHealthCheck() {
    try {
      const response = await fetch(`${this.baseUrl}/health`);
      return await response.json();
    } catch (error) {
      console.error('Error in translationHealthCheck:', error);
      throw error;
    }
  }

  // Get statistics about the RAG system
  async getStats() {
    try {
      const response = await fetch(`${this.baseUrl}/chat/stats`);
      return await response.json();
    } catch (error) {
      console.error('Error in getStats:', error);
      throw error;
    }
  }
}

// Export a singleton instance
const apiService = new ApiService();
export default apiService;

// Also export individual functions for direct use if needed
export { ApiService };
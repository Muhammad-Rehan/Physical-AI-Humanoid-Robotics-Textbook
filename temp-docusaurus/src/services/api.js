// API service for communicating with the RAG backend
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api/v1';

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
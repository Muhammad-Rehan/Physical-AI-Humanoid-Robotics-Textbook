// Test script to verify the API URL construction
const API_BASE_URL = 'https://muhammad1r-rag-chatbot.hf.space/api/v1';
// Extract base URL by removing the /chat part if it exists, or /v1 if it's at the end
const BASE_URL = API_BASE_URL.replace(/\/chat.*$/, '').replace(/\/v1$/, '');
const TRANSLATION_API_BASE_URL = `${BASE_URL}/v1`;

console.log('API_BASE_URL:', API_BASE_URL);
console.log('BASE_URL (after processing):', BASE_URL);
console.log('TRANSLATION_API_BASE_URL:', TRANSLATION_API_BASE_URL);

// Test with different URL patterns
const testUrls = [
  'https://muhammad1r-rag-chatbot.hf.space/api/v1',
  'https://muhammad1r-rag-chatbot.hf.space/api/v1/chat',
  'https://example.com/v1',
  'https://example.com/chat'
];

testUrls.forEach(url => {
  const base = url.replace(/\/chat.*$/, '').replace(/\/v1$/, '');
  const translationUrl = `${base}/v1`;
  console.log(`Original: ${url}`);
  console.log(`-> Translation API: ${translationUrl}`);
  console.log('---');
});
from typing import List, Dict, Optional
from ..models.query import QueryRequest, RAGResult
from .qdrant_service import QdrantService
from .embedding_service import EmbeddingService
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Try to import dashscope for Qwen text generation
try:
    import dashscope
    DASHSCOPE_AVAILABLE = True
except ImportError:
    DASHSCOPE_AVAILABLE = False
    dashscope = None

# Try to import zhipuai for GLM text generation
try:
    import zhipuai
    ZHIPUAI_AVAILABLE = True
except ImportError:
    ZHIPUAI_AVAILABLE = False
    zhipuai = None

# OpenRouter uses OpenAI library with their endpoint
OPENROUTER_AVAILABLE = True  # OpenRouter works through OpenAI library

class RAGService:
    def __init__(self):
        # Use the collection name from environment variable or default to 'book_content' for the textbook
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")
        self.qdrant_service = QdrantService(collection_name=collection_name)
        self.embedding_service = EmbeddingService()

        # Check for OpenRouter API key first for text generation (highest priority)
        openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
        use_openrouter_generation = os.getenv("USE_OPENROUTER_GENERATION", "false").lower() == "true"

        if use_openrouter_generation and openrouter_api_key and OPENROUTER_AVAILABLE:
            # OpenRouter will be used via OpenAI-compatible API
            self.use_openrouter = True
            self.use_glm = False
            self.use_qwen = False
            self.use_google = False
            self.model_type = "openrouter"
            print("Using OpenRouter for text generation")
        elif use_openrouter_generation and not OPENROUTER_AVAILABLE:
            print("OpenRouter generation requested but openrouter not installed")
            # Fall back to other providers
            self._initialize_other_providers()
        elif use_openrouter_generation and not openrouter_api_key:
            print("OpenRouter generation requested but no OPENROUTER_API_KEY found")
            # Fall back to other providers
            self._initialize_other_providers()
        else:
            # Fall back to other providers if OpenRouter is not configured
            self._initialize_other_providers()

    def _initialize_other_providers(self):
        # Check for GLM API key for text generation (second priority)
        glm_api_key = os.getenv("GLM_API_KEY")
        use_glm_generation = os.getenv("USE_GLM_GENERATION", "false").lower() == "true"

        if use_glm_generation and glm_api_key and ZHIPUAI_AVAILABLE:
            zhipuai.api_key = glm_api_key
            self.use_openrouter = False
            self.use_glm = True
            self.use_qwen = False
            self.use_google = False
            self.model_type = "glm"
            print("Using GLM for text generation")
        elif use_glm_generation and not ZHIPUAI_AVAILABLE:
            print("GLM generation requested but zhipuai not installed")
            # Fall back to other providers

            # Check for Qwen API key for text generation
            qwen_api_key = os.getenv("QWEN_API_KEY")
            use_qwen_generation = os.getenv("USE_QWEN_GENERATION", "false").lower() == "true"

            if use_qwen_generation and qwen_api_key and DASHSCOPE_AVAILABLE:
                dashscope.api_key = qwen_api_key
                self.use_openrouter = False
                self.use_glm = False
                self.use_qwen = True
                self.use_google = False
                self.model_type = "qwen"
                print("Using Qwen for text generation")
            else:
                # Initialize Google Generative AI
                google_api_key = os.getenv("GOOGLE_GEMINI_API_KEY")
                if google_api_key:
                    import google.generativeai as genai
                    genai.configure(api_key=google_api_key)
                    self.model = genai.GenerativeModel('gemini-pro')
                    self.use_openrouter = False
                    self.use_glm = False
                    self.use_google = True
                    self.use_qwen = False
                    self.model_type = "google"
                    print("Using Google Gemini for text generation")
                else:
                    print("No OpenRouter, GLM, Qwen or Google API key found, will return retrieved content directly")
                    self.use_openrouter = False
                    self.use_glm = False
                    self.use_google = False
                    self.use_qwen = False
                    self.model_type = "none"
                    self.model = None
        elif use_glm_generation and not glm_api_key:
            print("GLM generation requested but no GLM_API_KEY found")
            # Fall back to other providers

            # Check for Qwen API key for text generation
            qwen_api_key = os.getenv("QWEN_API_KEY")
            use_qwen_generation = os.getenv("USE_QWEN_GENERATION", "false").lower() == "true"

            if use_qwen_generation and qwen_api_key and DASHSCOPE_AVAILABLE:
                dashscope.api_key = qwen_api_key
                self.use_openrouter = False
                self.use_glm = False
                self.use_qwen = True
                self.use_google = False
                self.model_type = "qwen"
                print("Using Qwen for text generation")
            else:
                # Initialize Google Generative AI
                google_api_key = os.getenv("GOOGLE_GEMINI_API_KEY")
                if google_api_key:
                    import google.generativeai as genai
                    genai.configure(api_key=google_api_key)
                    self.model = genai.GenerativeModel('gemini-pro')
                    self.use_openrouter = False
                    self.use_glm = False
                    self.use_google = True
                    self.use_qwen = False
                    self.model_type = "google"
                    print("Using Google Gemini for text generation")
                else:
                    print("No OpenRouter, GLM, Qwen or Google API key found, will return retrieved content directly")
                    self.use_openrouter = False
                    self.use_glm = False
                    self.use_google = False
                    self.use_qwen = False
                    self.model_type = "none"
                    self.model = None
        else:
            # Check for Qwen API key for text generation (third priority)
            qwen_api_key = os.getenv("QWEN_API_KEY")
            use_qwen_generation = os.getenv("USE_QWEN_GENERATION", "false").lower() == "true"

            if use_qwen_generation and qwen_api_key and DASHSCOPE_AVAILABLE:
                dashscope.api_key = qwen_api_key
                self.use_openrouter = False
                self.use_glm = False
                self.use_qwen = True
                self.use_google = False
                self.model_type = "qwen"
                print("Using Qwen for text generation")
            else:
                # Initialize Google Generative AI (fourth priority)
                google_api_key = os.getenv("GOOGLE_GEMINI_API_KEY")
                if google_api_key:
                    import google.generativeai as genai
                    genai.configure(api_key=google_api_key)
                    self.model = genai.GenerativeModel('gemini-pro')
                    self.use_openrouter = False
                    self.use_glm = False
                    self.use_google = True
                    self.use_qwen = False
                    self.model_type = "google"
                    print("Using Google Gemini for text generation")
                else:
                    print("No OpenRouter, GLM, Qwen or Google API key found, will return retrieved content directly")
                    self.use_openrouter = False
                    self.use_glm = False
                    self.use_google = False
                    self.use_qwen = False
                    self.model_type = "none"
                    self.model = None

    def process_query(self, request: QueryRequest) -> RAGResult:
        """
        Process a user query using RAG methodology
        """
        if request.selected_text:
            # Handle selected text mode - only use the provided selected text
            return self._process_selected_text_query(request)
        else:
            # Handle general question mode - search in the knowledge base
            return self._process_general_query(request)

    def _process_selected_text_query(self, request: QueryRequest) -> RAGResult:
        """
        Process query when user has selected specific text
        """
        # In selected text mode, we only use the provided selected text
        relevant_chunks = [{
            'text': request.selected_text,
            'url': 'selected_text',
            'title': 'User Selected Text'
        }]

        if self.use_openrouter:
            # Generate answer based on selected text using OpenRouter
            prompt = f"""
            Answer the following question based ONLY on the provided selected text.
            Do not use any external knowledge or make assumptions beyond what's in the text.
            If the answer cannot be found in the provided text, say so explicitly.

            Selected Text: {request.selected_text}

            Question: {request.question}

            Answer:
            """

            try:
                # Use OpenRouter's API (using OpenAI-compatible interface)
                import openai
                openai.api_key = os.getenv("OPENROUTER_API_KEY")
                openai.base_url = "https://openrouter.ai/api/v1"

                response = openai.chat.completions.create(
                    model=os.getenv("OPENROUTER_MODEL", "microsoft/wizardlm-2-8x22b"),  # Use configured model or default
                    messages=[
                        {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                        {"role": "user", "content": prompt}
                    ],
                    temperature=0.1,  # Low temperature for factual accuracy
                    max_tokens=1000
                )

                # Handle the response - check if it has the expected structure
                if hasattr(response, 'choices') and response.choices and len(response.choices) > 0:
                    if hasattr(response.choices[0], 'message') and hasattr(response.choices[0].message, 'content'):
                        answer = response.choices[0].message.content
                    else:
                        raise AttributeError("Response choice message does not have expected content structure")
                else:
                    # Check if there's an error in the response
                    if hasattr(response, 'error'):
                        raise Exception(f"OpenRouter API returned an error: {response.error}")
                    else:
                        raise AttributeError("Response does not have expected choices structure")

                # Prepend note about selected text mode
                answer = f"Answering from selected text only:\n\n{answer}"
            except Exception as e:
                # Fallback to other providers if OpenRouter fails
                try:
                    # Try a different OpenRouter model if the default fails
                    import openai
                    openai.api_key = os.getenv("OPENROUTER_API_KEY")
                    openai.base_url = "https://openrouter.ai/api/v1"

                    response = openai.chat.completions.create(
                        model="openchat/openchat-7b",  # Alternative free model
                        messages=[
                            {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                            {"role": "user", "content": prompt}
                        ],
                        temperature=0.1,
                        max_tokens=1000
                    )

                    # Handle the response - check if it has the expected structure
                    if hasattr(response, 'choices') and response.choices and len(response.choices) > 0:
                        if hasattr(response.choices[0], 'message') and hasattr(response.choices[0].message, 'content'):
                            answer = response.choices[0].message.content
                        else:
                            raise AttributeError("Response choice message does not have expected content structure")
                    else:
                        # Check if there's an error in the response
                        if hasattr(response, 'error'):
                            raise Exception(f"OpenRouter API returned an error: {response.error}")
                        else:
                            raise AttributeError("Response does not have expected choices structure")

                    # Prepend note about selected text mode
                    answer = f"Answering from selected text only:\n\n{answer}"
                except Exception as e2:
                    # Fallback if OpenRouter generation fails
                    answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: OpenRouter generation error - {str(e2)}"
        elif self.use_glm:
            # Generate answer based on selected text using GLM
            prompt = f"""
            Answer the following question based ONLY on the provided selected text.
            Do not use any external knowledge or make assumptions beyond what's in the text.
            If the answer cannot be found in the provided text, say so explicitly.

            Selected Text: {request.selected_text}

            Question: {request.question}

            Answer:
            """

            try:
                from zhipuai import ZhipuAI
                client = ZhipuAI(api_key=zhipuai.api_key)  # Create client instance

                response = client.chat.completions.create(
                    model="GLM-4.5",  # Using GLM-4 model (this is the standard model name)
                    messages=[
                        {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                        {"role": "user", "content": prompt}
                    ],
                    temperature=0.1,  # Low temperature for factual accuracy
                    max_tokens=1000
                )

                answer = response.choices[0].message.content
                # Prepend note about selected text mode
                answer = f"Answering from selected text only:\n\n{answer}"
            except Exception as e:
                # Try alternative model name if glm-4 is not available
                try:
                    from zhipuai import ZhipuAI
                    client = ZhipuAI(api_key=zhipuai.api_key)

                    response = client.chat.completions.create(
                        model="GLM-4-Plus",  # Alternative GLM model name
                        messages=[
                            {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                            {"role": "user", "content": prompt}
                        ],
                        temperature=0.1,
                        max_tokens=1000
                    )

                    answer = response.choices[0].message.content
                    # Prepend note about selected text mode
                    answer = f"Answering from selected text only:\n\n{answer}"
                except Exception as e2:
                    # If both models fail, try the most basic one
                    try:
                        from zhipuai import ZhipuAI
                        client = ZhipuAI(api_key=zhipuai.api_key)

                        response = client.chat.completions.create(
                            model="GLM-4.6V-Flash",  # Fallback to turbo model
                            messages=[
                                {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                                {"role": "user", "content": prompt}
                            ],
                            temperature=0.1,
                            max_tokens=1000
                        )

                        answer = response.choices[0].message.content
                        # Prepend note about selected text mode
                        answer = f"Answering from selected text only:\n\n{answer}"
                    except Exception as e3:
                        # Fallback if GLM generation fails
                        answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: GLM generation error - {str(e3)}"
        elif self.use_qwen:
            # Generate answer based on selected text using Qwen
            prompt = f"""
            Answer the following question based ONLY on the provided selected text.
            Do not use any external knowledge or make assumptions beyond what's in the text.
            If the answer cannot be found in the provided text, say so explicitly.

            Selected Text: {request.selected_text}

            Question: {request.question}

            Answer:
            """

            try:
                response = dashscope.Generation.call(
                    model="qwen-max",  # Using Qwen's capable model
                    prompt=prompt,
                    temperature=0.1,  # Low temperature for factual accuracy
                    max_tokens=1000
                )

                if response.status_code == 200:
                    answer = response.output.text
                    # Prepend note about selected text mode
                    answer = f"Answering from selected text only:\n\n{answer}"
                else:
                    # Fallback if Qwen API fails
                    answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: Qwen API error - {response.code}: {response.message}"
            except Exception as e:
                # Fallback if Qwen generation fails
                answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: Qwen generation error - {str(e)}"
        elif self.use_google:
            # Generate answer based on selected text using Google
            prompt = f"""
            Answer the following question based ONLY on the provided selected text.
            Do not use any external knowledge or make assumptions beyond what's in the text.
            If the answer cannot be found in the provided text, say so explicitly.

            Selected Text: {request.selected_text}

            Question: {request.question}

            Answer:
            """

            try:
                response = self.model.generate_content(
                    prompt,
                    generation_config={
                        "temperature": 0.1,  # Low temperature for factual accuracy
                        "max_output_tokens": 1000
                    }
                )

                answer = response.text

                # Prepend note about selected text mode
                answer = f"Answering from selected text only:\n\n{answer}"
            except Exception as e:
                # Fallback if Google generation fails
                answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: Google generation error - {str(e)}"
        else:
            # Without Google, Qwen, or GLM, just return the selected text with a note
            answer = f"Selected text mode: Here is the selected text you provided:\n\n{request.selected_text}\n\nQuestion: {request.question}\n\nNote: Without Google, Qwen, or GLM API key, full answer generation is not available."

        return RAGResult(
            answer=answer,
            relevant_chunks=relevant_chunks,
            sources=["Selected Text"],
            query_embedding=None
        )

    def _process_general_query(self, request: QueryRequest) -> RAGResult:
        """
        Process general query by searching in the knowledge base
        """
        # Generate embedding for the query
        query_embedding = self.embedding_service.embed_text(request.question)

        # Search for relevant chunks in the vector database
        relevant_chunks = self.qdrant_service.search_similar(
            query_embedding,
            limit=request.context_window
        )

        if not relevant_chunks:
            # If no relevant chunks found, return appropriate response
            return RAGResult(
                answer="I couldn't find relevant information in the textbook to answer your question.",
                relevant_chunks=[],
                sources=[],
                query_embedding=query_embedding
            )

        # Prepare context from relevant chunks
        context_texts = [chunk['text'] for chunk in relevant_chunks]
        context = "\n\n".join(context_texts)

        if self.use_openrouter:
            # Generate prompt for OpenRouter
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the following question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make assumptions beyond what's in the context.
            If the question is about a topic not covered in the context, explicitly state that the information is not available in the textbook.
            Do not attempt to answer questions that are outside the scope of the provided context.

            Context:
            {context}

            Question: {request.question}

            Answer (include source citations if possible, or state that the information is not in the textbook):
            """

            try:
                # Use OpenRouter's API (using OpenAI-compatible interface)
                import openai
                openai.api_key = os.getenv("OPENROUTER_API_KEY")
                openai.base_url = "https://openrouter.ai/api/v1"

                response = openai.chat.completions.create(
                    model=os.getenv("OPENROUTER_MODEL", "microsoft/wizardlm-2-8x22b"),  # Use configured model or default
                    messages=[
                        {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                        {"role": "user", "content": prompt}
                    ],
                    temperature=0.1,  # Low temperature for factual accuracy
                    max_tokens=1000
                )

                # Handle the response - check if it has the expected structure
                if hasattr(response, 'choices') and response.choices and len(response.choices) > 0:
                    if hasattr(response.choices[0], 'message') and hasattr(response.choices[0].message, 'content'):
                        answer = response.choices[0].message.content
                    else:
                        raise AttributeError("Response choice message does not have expected content structure")
                else:
                    # Check if there's an error in the response
                    if hasattr(response, 'error'):
                        raise Exception(f"OpenRouter API returned an error: {response.error}")
                    else:
                        raise AttributeError("Response does not have expected choices structure")
            except Exception as e:
                # Fallback to other providers if OpenRouter fails
                try:
                    # Try a different OpenRouter model if the default fails
                    import openai
                    openai.api_key = os.getenv("OPENROUTER_API_KEY")
                    openai.base_url = "https://openrouter.ai/api/v1"

                    response = openai.chat.completions.create(
                        model="openchat/openchat-7b",  # Alternative free model
                        messages=[
                            {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                            {"role": "user", "content": prompt}
                        ],
                        temperature=0.1,
                        max_tokens=1000
                    )

                    # Handle the response - check if it has the expected structure
                    if hasattr(response, 'choices') and response.choices and len(response.choices) > 0:
                        if hasattr(response.choices[0], 'message') and hasattr(response.choices[0].message, 'content'):
                            answer = response.choices[0].message.content
                        else:
                            raise AttributeError("Response choice message does not have expected content structure")
                    else:
                        # Check if there's an error in the response
                        if hasattr(response, 'error'):
                            raise Exception(f"OpenRouter API returned an error: {response.error}")
                        else:
                            raise AttributeError("Response does not have expected choices structure")
                except Exception as e2:
                    # Fallback if OpenRouter generation fails
                    answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: OpenRouter generation error - {str(e2)}. The relevant information from the textbook is shown above."
        elif self.use_glm:
            # Generate prompt for GLM
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the following question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make assumptions beyond what's in the context.
            If the question is about a topic not covered in the context, explicitly state that the information is not available in the textbook.
            Do not attempt to answer questions that are outside the scope of the provided context.

            Context:
            {context}

            Question: {request.question}

            Answer (include source citations if possible, or state that the information is not in the textbook):
            """

            try:
                from zhipuai import ZhipuAI
                client = ZhipuAI(api_key=zhipuai.api_key)  # Create client instance

                response = client.chat.completions.create(
                    model="GLM-4.5V",  # Using GLM-4 model (this is the standard model name)
                    messages=[
                        {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                        {"role": "user", "content": prompt}
                    ],
                    temperature=0.1,  # Low temperature for factual accuracy
                    max_tokens=1000
                )

                answer = response.choices[0].message.content
            except Exception as e:
                # Try alternative model name if glm-4 is not available
                try:
                    from zhipuai import ZhipuAI
                    client = ZhipuAI(api_key=zhipuai.api_key)

                    response = client.chat.completions.create(
                        model="GLM-4-Plus",  # Alternative GLM model name
                        messages=[
                            {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                            {"role": "user", "content": prompt}
                        ],
                        temperature=0.1,
                        max_tokens=1000
                    )

                    answer = response.choices[0].message.content
                except Exception as e2:
                    # If both models fail, try the most basic one
                    try:
                        from zhipuai import ZhipuAI
                        client = ZhipuAI(api_key=zhipuai.api_key)

                        response = client.chat.completions.create(
                            model="GLM-4.6V-Flash",  # Fallback to turbo model
                            messages=[
                                {"role": "system", "content": "You are an expert assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based only on the provided context."},
                                {"role": "user", "content": prompt}
                            ],
                            temperature=0.1,
                            max_tokens=1000
                        )

                        answer = response.choices[0].message.content
                    except Exception as e3:
                        # Fallback if GLM generation fails
                        answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: GLM generation error - {str(e3)}. The relevant information from the textbook is shown above."
        elif self.use_qwen:
            # Generate prompt for Qwen
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the following question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make assumptions beyond what's in the context.
            If the question is about a topic not covered in the context, explicitly state that the information is not available in the textbook.
            Do not attempt to answer questions that are outside the scope of the provided context.

            Context:
            {context}

            Question: {request.question}

            Answer (include source citations if possible, or state that the information is not in the textbook):
            """

            try:
                response = dashscope.Generation.call(
                    model="qwen-max",  # Using Qwen's capable model
                    prompt=prompt,
                    temperature=0.1,  # Low temperature for factual accuracy
                    max_tokens=1000
                )

                if response.status_code == 200:
                    answer = response.output.text
                else:
                    # Fallback if Qwen API fails
                    answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: Qwen API error - {response.code}: {response.message}. The relevant information from the textbook is shown above."
            except Exception as e:
                # Fallback if Qwen generation fails
                answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: Qwen generation error - {str(e)}. The relevant information from the textbook is shown above."
        elif self.use_google:
            # Generate prompt for the LLM
            prompt = f"""
            You are an expert assistant for the Physical AI & Humanoid Robotics textbook.
            Answer the following question based ONLY on the provided context from the textbook.
            Do not use any external knowledge or make assumptions beyond what's in the context.
            If the question is about a topic not covered in the context, explicitly state that the information is not available in the textbook.
            Do not attempt to answer questions that are outside the scope of the provided context.

            Context:
            {context}

            Question: {request.question}

            Answer (include source citations if possible, or state that the information is not in the textbook):
            """

            try:
                response = self.model.generate_content(
                    prompt,
                    generation_config={
                        "temperature": 0.1,  # Low temperature for factual accuracy
                        "max_output_tokens": 1000
                    }
                )

                answer = response.text
            except Exception as e:
                # Fallback if Google generation fails
                answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: Google generation error - {str(e)}. The relevant information from the textbook is shown above."
        else:
            # Without Google, Qwen, or GLM, return the retrieved context with the question
            answer = f"Question: {request.question}\n\nRetrieved context:\n{context}\n\nNote: Without Google, Qwen, or GLM API key, full answer generation is not available. The relevant information from the textbook is shown above."

        # Extract sources from relevant chunks
        sources = []
        for chunk in relevant_chunks:
            source = chunk.get('url', 'Unknown source')
            if source and source != 'Unknown source':
                sources.append(source)

        return RAGResult(
            answer=answer,
            relevant_chunks=relevant_chunks,
            sources=sources,
            query_embedding=query_embedding
        )

    def get_answer_with_sources(self, request: QueryRequest) -> RAGResult:
        """
        Main method to get answer with proper source citations
        """
        result = self.process_query(request)

        # Add source citations to the answer if sources exist
        if result.sources:
            result.answer += f"\n\nSources: {', '.join(result.sources[:3])}"  # Limit to first 3 sources

        return result
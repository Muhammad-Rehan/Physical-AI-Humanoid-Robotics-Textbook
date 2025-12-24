from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv
import os

# Import the chat and translation routers
from src.api.v1.endpoints.chat import router as chat_router
from src.api.v1.translation import router as translation_router

# Load environment variables from .env file
load_dotenv()

app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation API for Physical AI & Humanoid Robotics Textbook",
    version="1.0.0"
)

# Add CORS middleware to allow requests from the Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with your specific domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the chat and translation routers
app.include_router(chat_router, prefix="/api/v1")
app.include_router(translation_router, prefix="/api/v1")

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running!"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
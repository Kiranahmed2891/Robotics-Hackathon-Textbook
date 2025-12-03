# backend/utils/generator.py

import os
from google import genai
from typing import List, Dict

# Configuration
# LLM_API_KEY is read from the .env file (T006)
LLM_API_KEY = os.getenv("LLM_API_KEY") 
LLM_MODEL = "gemini-2.5-flash"

# Global client instance
_client = None

def get_llm_client() -> genai.Client:
    """
    Initializes and returns the Gemini API client.
    """
    global _client
    
    if _client is None:
        if not LLM_API_KEY:
            # Note: This check relies on the calling context loading dotenv, which is done in qdrant_service.py
            if not LLM_API_KEY:
                 raise ValueError("LLM_API_KEY environment variable not set.")
        
        print(f"Initializing Gemini Client...")
        # API Key ko direct pass karna theek hai, but genai library environment variable se bhi le leti hai.
        _client = genai.Client(api_key=LLM_API_KEY)
            
    return _client

def generate_rag_answer(query: str, context: List[Dict]) -> str:
    """
    Uses the retrieved context to generate a refined answer to the user's query using the LLM.
    """
    client = get_llm_client()
    
    # 1. Format the context into a single string
    context_text = "\n---\n".join([c['text'] for c in context])

    # 🚨 FIX: Agar koi context nahi mila to seedhe error return karein. 
    # (Yeh tab hoga agar Retrieval ne खाली list return ki ho)
    if not context_text.strip():
        # Ye message wahi hona chahiye jo aapka prompt LLM se expect karta hai, 
        # lekin yahan hum seedhe return kar sakte hain.
        return "The provided textbook context does not contain the answer to your question."


    # 2. Define the System Prompt
    system_prompt_text = (
        "You are an expert Q&A assistant for a technical textbook. "
        "Your goal is to answer the user's question ONLY based on the provided CONTEXT. "
        "If the answer is not found in the context, clearly state: 'The provided textbook context does not contain the answer to your question.' "
        "Do not use external knowledge. Keep the response concise and accurate."
    )
    
    # 3. Construct the full user prompt containing context and query
    user_prompt_text = (
        f"CONTEXT:\n---\n{context_text}\n---\n\n"
        f"USER QUESTION: {query}"
    )

    # 4. Construct the contents list (Best Practice: System instruction with Role)
    contents = [
        {"role": "system", "parts": [{"text": system_prompt_text}]},
        {"role": "user", "parts": [{"text": user_prompt_text}]},
    ]

    try:
        response = client.models.generate_content(
            model=LLM_MODEL,
            contents=contents, # Structured contents list used here
        )
        return response.text
    
    except Exception as e:
        print(f"LLM Generation Error: {e}")
        return f"Error: Could not generate a response from the LLM. Check API key and connection: {e}"
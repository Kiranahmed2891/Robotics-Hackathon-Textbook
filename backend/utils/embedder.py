# backend/utils/embedder.py

import os
from sentence_transformers import SentenceTransformer
from typing import List

# Embedding Model Configuration
# Note: Use a model that outputs 384 dimensions to match the Qdrant collection setup (T1.4)
EMBEDDING_MODEL_NAME = "all-MiniLM-L6-v2" 

# Global variable to hold the initialized model instance
_model = None

def get_embedding_model() -> SentenceTransformer:
    """
    Initializes and returns the SentenceTransformer model.
    The model is loaded only once (singleton pattern) for efficiency.
    """
    global _model
    
    if _model is None:
        print(f"Loading embedding model: {EMBEDDING_MODEL_NAME}...")
        try:
            # We assume the user has run 'pip install -r requirements.txt'
            _model = SentenceTransformer(EMBEDDING_MODEL_NAME)
            print("Embedding model loaded successfully.")
        except Exception as e:
            print(f"Error loading SentenceTransformer: {e}")
            raise RuntimeError(f"Could not load embedding model {EMBEDDING_MODEL_NAME}.") from e
            
    return _model

def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Converts a list of text strings into their corresponding numerical vector embeddings.
    """
    model = get_embedding_model()
    
    # Encode the texts (this handles the actual conversion)
    embeddings = model.encode(texts, convert_to_tensor=False)
    
    # Convert numpy array output to a list of lists (required for Qdrant storage)
    return embeddings.tolist()


# Example Test (Optional: Requires the 'sentence-transformers' library to be installed)
# if __name__ == "__main__":
#     texts_to_embed = ["What is the capital of France?", "The quick brown fox jumps over the lazy dog."]
#     vectors = generate_embeddings(texts_to_embed)
#     print(f"Generated {len(vectors)} vectors.")
#     if vectors:
#         print(f"Vector size: {len(vectors[0])}")
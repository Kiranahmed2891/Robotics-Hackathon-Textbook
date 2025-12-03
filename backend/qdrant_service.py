# backend/qdrant_service.py (FINAL GLOBAL INSTANCE FIX + DOTENV LOAD)

import os
from qdrant_client import QdrantClient, models
from qdrant_client.models import Distance, VectorParams, CollectionStatus
from dotenv import load_dotenv # ✅ FIX 1: Import dotenv

load_dotenv() # ✅ FIX 1: Load environment variables

# --- Global Configuration ---
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "textbook_collection")
VECTOR_DIMENSION = 384 

# 🚨 FIX 2: Global variable to hold the single Qdrant client instance
QDRANT_CLIENT_INSTANCE = None 

# --- Qdrant Client Initialization ---
def get_qdrant_client() -> QdrantClient:
    """
    Returns the single, globally managed Qdrant client instance.
    """
    global QDRANT_CLIENT_INSTANCE
    
    if QDRANT_CLIENT_INSTANCE is not None:
        return QDRANT_CLIENT_INSTANCE

    # URL aur API key ab load ho chuke hain
    if QDRANT_URL and QDRANT_API_KEY:
        print(f"INFO: Connecting to remote Qdrant at {QDRANT_URL}")
        client = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY,
            timeout=20
        )
    else:
        print("Warning: QDRANT_URL or QDRANT_API_KEY is missing. Using in-memory client for testing.")
        client = QdrantClient(":memory:")
        
    QDRANT_CLIENT_INSTANCE = client
    return client


# --- Collection Initialization ---
def initialize_collection(client: QdrantClient):
    """
    Checks if the collection exists, and creates it if it doesn't.
    """
    try:
        collections = client.get_collections().collections
        
        if COLLECTION_NAME in [c.name for c in collections]:
            print(f"Collection '{COLLECTION_NAME}' already exists.")
            return

        client.recreate_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=VECTOR_DIMENSION, distance=Distance.COSINE),
        )
        print(f"Collection '{COLLECTION_NAME}' created successfully with size {VECTOR_DIMENSION}.")

    except Exception as e:
        print(f"Error initializing Qdrant collection: {e}")
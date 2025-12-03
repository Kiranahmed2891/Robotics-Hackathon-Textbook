from typing import List, Dict, Any
from .embedder import generate_embeddings
from ..qdrant_service import get_qdrant_client, COLLECTION_NAME
from qdrant_client.models import SearchParams

RETRIEVAL_LIMIT = 5 # Number of chunks to retrieve

def retrieve_relevant_chunks(query_text: str) -> List[Dict[str, Any]]:
    """
    Converts the query into a vector and searches the Qdrant collection for relevant chunks.
    """
    # 1. Embed the query text
    query_vector = generate_embeddings([query_text])[0] 

    # 2. Search Qdrant
    client = get_qdrant_client()
    
    # Perform the search operation
    search_result = client.query_points( 
        collection_name=COLLECTION_NAME,
        query=query_vector, 
        limit=RETRIEVAL_LIMIT,
        with_payload=True, 
        with_vectors=False, # Vectors excluded
        search_params=SearchParams(
            hnsw_ef=128, 
            exact=False
        )
    )

    # 3. Format results (Robust handling for unexpected return types)
    retrieved_chunks = []
    for hit in search_result:
        
        # Initialize default values
        payload = None
        score = None
        chunk_text = "Content missing"
        
        # 3a. Check for standard Qdrant ScoredPoint object
        if hasattr(hit, 'payload') and hasattr(hit, 'score'):
            payload = hit.payload
            score = hit.score
        
        # 3b. Fallback: Check if it's a tuple containing the ScoredPoint object (less common, but handles specific errors)
        elif isinstance(hit, (list, tuple)) and len(hit) > 0 and hasattr(hit[0], 'payload'):
            hit_obj = hit[0]
            payload = hit_obj.payload
            score = hit_obj.score

        # 3c. Process if payload was successfully extracted
        if payload is not None:
            chunk_text = payload.get("content_chunk", "Content missing")
            
            retrieved_chunks.append({
                "text": chunk_text, 
                "metadata": payload,
                "score": score
            })
        else:
            # If nothing worked, log the issue and skip the item
            # print(f"Warning: Could not extract payload from hit of type {type(hit)}. Skipping.")
            continue
        
    return retrieved_chunks
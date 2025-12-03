# backend/main.py (FINAL ROBUSTNESS FIX)

import os
import uuid 
from dotenv import load_dotenv
from fastapi import FastAPI
from pydantic import BaseModel
from qdrant_client import models 
from fastapi.middleware.cors import CORSMiddleware 

# Import the utility modules (Phase 2 & 3 Dependencies)
from .utils.loader import load_markdown_documents
from .utils.chunker import split_documents_into_chunks
from .utils.embedder import generate_embeddings
from .qdrant_service import get_qdrant_client, initialize_collection, COLLECTION_NAME # initialize_collection ab sirf startup mein call hoga
from .utils.retriever import retrieve_relevant_chunks 
from .utils.generator import generate_rag_answer 

# Load environment variables from .env file
load_dotenv()

app = FastAPI(
    title="RAG Chatbot Backend",
    description="FastAPI service for Retrieval-Augmented Generation.",
    version="1.0.0"
)

# ----------------------------------------------------
# 🛡️ CORS FIX 
origins = [
    "http://localhost",
    "http://localhost:3000",
    "http://127.0.0.1",
    "http://127.0.0.1:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
# ----------------------------------------------------


# --- Pydantic Models ---
class IngestRequest(BaseModel):
    document_path: str
    metadata: dict = {}

class ChatRequest(BaseModel):
    query: str


# --- T1.4: Startup Event for Qdrant Initialization ---
@app.on_event("startup")
def startup_event():
    """
    Initializes the Qdrant client and ensures the collection exists when the server starts.
    """
    try:
        qdrant_client = get_qdrant_client()
        initialize_collection(qdrant_client)
    except Exception as e:
        print(f"FATAL: Could not connect or initialize Qdrant. Server will run without vector store. Error: {e}")


# --- T2.4: FULL Ingestion Pipeline Endpoint (CLEANED) ---
@app.post("/api/v1/ingest", tags=["Ingestion"])
def ingest_documents_to_qdrant():
    """
    Executes the full RAG ingestion pipeline: Load -> Chunk -> Embed -> Store (Qdrant).
    """
    try:
        # 1. Load Documents
        documents = load_markdown_documents()
        if not documents:
            return {"status": "failure", "message": "No documents found to ingest."}

        # 2. Chunk Documents
        chunks = split_documents_into_chunks(documents)

        # 3. Embed Chunks
        texts = [chunk["text"] for chunk in chunks]
        vectors = generate_embeddings(texts)

        # 4. Prepare Points for Qdrant
        points = []
        for i, vector in enumerate(vectors):
            chunk = chunks[i]
            point_id = str(uuid.uuid4()) 
            
            # Ensure 'content_chunk' (for RAG retrieval) is stored in metadata/payload
            chunk["metadata"]["content_chunk"] = chunk["text"]
            
            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=vector,
                    payload=chunk["metadata"],
                )
            )

        # 5. Store in Qdrant
        qdrant_client = get_qdrant_client()
        
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
            wait=True
        )

        return {
            "status": "success",
            "message": f"Successfully ingested {len(points)} chunks into Qdrant collection '{COLLECTION_NAME}'.",
            "chunks_ingested": len(points)
        }

    except Exception as e:
        print(f"Ingestion Error: {e}")
        return {"status": "failure", "message": f"Ingestion failed due to an internal error: {e}"}


# --- T3.3: RAG Chat Endpoint (ROBUST RETRIEVAL CHECK) ---
@app.post("/api/v1/chat", tags=["RAG Chat"])
def chat_with_document(request: ChatRequest):
    """
    Performs the full RAG workflow: Retrieve context, augment prompt, and generate answer.
    """
    query = request.query
    
    if not query:
        return {"answer": "Please provide a query.", "context": [], "status": "success"}

    try:
        # 1. Retrieval (T3.1)
        relevant_chunks = retrieve_relevant_chunks(query)

        # 🚨 FIX: Ensure we have chunks. Since there's no score filtration here,
        # we assume if the list is empty, nothing relevant was found.
        if not relevant_chunks:
            # Note: This matches the default message expected by the frontend.
            return {"answer": "Could not find relevant context in the textbook.", "context": [], "status": "success"}

        # 2. Generation (T3.2)
        # We send ALL retrieved chunks to the generator to maximize chances of getting an answer.
        final_answer = generate_rag_answer(query, relevant_chunks)
        
        # 3. Format Context for Response
        context_response = [
            {
                "text_snippet": chunk["metadata"].get("content_chunk", "Content missing")[:100] + "...", 
                "source": chunk["metadata"].get("source_path", "N/A"),
                "chapter": chunk["metadata"].get("chapter_name", "N/A"),
                "score": chunk["score"]
            }
            for chunk in relevant_chunks
        ]

        return {
            "answer": final_answer,
            "context": context_response,
            "status": "success"
        }

    except Exception as e:
        print(f"Chat Endpoint Error: {e}")
        return {"answer": f"An error occurred during processing: {e}", "context": [], "status": "failure"}


# --- T1.3: Health Check Endpoint ---
@app.get("/health", tags=["System"])
def health_check():
    """
    Checks if the API service is running.
    """
    return {"status": "ok", "message": "FastAPI application is healthy"}
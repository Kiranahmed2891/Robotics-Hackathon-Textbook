# Feature: Integrated RAG Chatbot Tasks

## Phase 1: Setup (Infrastructure)

- [ ] T001 Create backend directory and initial files: `backend/requirements.txt`, `backend/main.py`
- [ ] T002 Create frontend directory and initial files: `frontend/package.json`, `frontend/src/index.tsx`
- [ ] T003 Create `backend/Dockerfile` for FastAPI application
- [ ] T004 Create `frontend/Dockerfile` for React application
- [ ] T005 Create `docker-compose.yml` for multi-service deployment
- [ ] T006 Configure environment variables in `.env` file for backend (LLM API key, Embedding API key, ChromaDB settings)
- [ ] T007 Configure environment variables in `.env` file for frontend (Backend API URL)

## Phase 2: Foundational (Blocking prerequisites for all user stories)

- [ ] T008 [P] Install backend dependencies in `backend/requirements.txt` (FastAPI, Uvicorn, LangChain, OpenAI, ChromaDB client)
- [ ] T009 [P] Install frontend dependencies in `frontend/package.json` (React, ReactDOM, TypeScript, Axios)
- [ ] T010 Implement basic FastAPI application structure in `backend/main.py`
- [ ] T011 Implement common utility functions (e.g., logging) in `backend/utils.py`
- [ ] T012 Initialize and configure OpenAI LLM client in `backend/llm_client.py`
- [ ] T013 Initialize and configure OpenAI Embedding client in `backend/embedding_client.py`
- [ ] T014 Initialize and configure ChromaDB client in `backend/vector_db.py`
- [ ] T015 Define Pydantic models for API requests and responses in `backend/models.py`

## Phase 3: User Story 1 - Ask a Question & Get a Contextual Answer (Priority: P1)

**Story Goal**: As a user, I want to ask questions about the ingested documents and receive accurate, contextually relevant answers based on the content of those documents.
**Independent Test**: Ingest a set of documents, then ask a question that can be answered by those documents, and verify the response's accuracy and source attribution.

- [ ] T016 [US1] Implement query embedding logic using `backend/embedding_client.py`
- [ ] T017 [US1] Implement vector retrieval from ChromaDB using `backend/vector_db.py`
- [ ] T018 [US1] Implement context integration with LLM prompt using `backend/llm_client.py`
- [ ] T019 [US1] Implement LLM call and response generation using `backend/llm_client.py`
- [ ] T020 [US1] Create `/chat` API endpoint in `backend/main.py` that integrates RAG logic
- [ ] T021 [P] [US1] Create `frontend/src/components/Chat.tsx` for the basic chat UI
- [ ] T022 [US1] Implement sending user queries from `frontend/src/components/Chat.tsx` to the backend `/chat` endpoint
- [ ] T023 [US1] Implement displaying LLM responses in `frontend/src/components/Chat.tsx`

## Phase 4: User Story 2 - Ingest New Documents (Priority: P2)

**Story Goal**: As a user, I want to upload new documents (e.g., Markdown, PDF, TXT) to expand the chatbot's knowledge base, so it can answer questions based on the new content.
**Independent Test**: Upload a new document, confirm successful ingestion via API response, and then ask a question related to the newly ingested document, verifying the chatbot uses this new information.

- [ ] T024 [P] [US2] Implement document parsing for Markdown in `backend/document_parser.py`
- [ ] T025 [P] [US2] Implement document parsing for PDF in `backend/document_parser.py`
- [ ] T026 [P] [US2] Implement document parsing for TXT in `backend/document_parser.py`
- [ ] T027 [US2] Implement text chunking logic in `backend/text_splitter.py`
- [ ] T028 [US2] Implement chunk embedding using `backend/embedding_client.py`
- [ ] T029 [US2] Implement storage of embeddings and metadata in ChromaDB using `backend/vector_db.py`
- [ ] T030 [US2] Create `/ingest` API endpoint in `backend/main.py` for document upload
- [ ] T031 [US2] Create ingestion script (`backend/ingest.py`) for local document processing

## Phase 5: User Story 3 - View Chat History (Priority: P3)

**Story Goal**: As a user, I want to see my previous questions and the chatbot's responses within a session, so I can keep track of the conversation flow.
**Independent Test**: Have a short conversation with the chatbot and then verify that all questions and answers are displayed in chronological order.

- [ ] T032 [US3] Implement chat session management (e.g., storing messages in memory or a simple database) in `backend/session_manager.py`
- [ ] T033 [US3] Modify `/chat` API in `backend/main.py` to accept and return `session_id`
- [ ] T034 [US3] Implement retrieval of chat history by `session_id` in `backend/session_manager.py`
- [ ] T035 [US3] Integrate chat history display in `frontend/src/components/Chat.tsx`
- [ ] T036 [US3] Implement passing `session_id` with chat requests from `frontend/src/components/Chat.tsx`

## Final Phase: Polish & Cross-Cutting Concerns

- [ ] T037 Implement comprehensive error handling for all backend API endpoints
- [ ] T038 Add source attribution (document snippets) to chat responses in `backend/main.py` and display in `frontend/src/components/Chat.tsx`
- [ ] T039 Refine Docker configurations and add instructions for local development/deployment
- [ ] T040 Implement basic unit tests for core RAG logic in `backend/tests/test_rag.py`
- [ ] T041 Implement basic integration tests for API endpoints in `backend/tests/test_api.py`

## Dependencies

- Phase 1 tasks must be completed before proceeding to subsequent phases.
- Phase 2 tasks must be completed before user story phases.
- User Story 1 (P1) is independent of P2 and P3 for its core functionality, but P2 (Ingestion) is a prerequisite for P1 to be fully functional with user-provided documents. P3 (Chat History) is dependent on P1 for chat interactions.

## Parallel Execution Examples

- `T003`, `T004` (Dockerfiles) can be worked on in parallel.
- `T008`, `T009` (installing dependencies) can be worked on in parallel.
- `T024`, `T025`, `T026` (document parsers) can be developed in parallel.

## Implementation Strategy

The implementation will follow an MVP-first approach, focusing on delivering User Story 1 (P1) functionality as quickly as possible. Subsequent user stories (P2, P3) will be implemented incrementally, building upon the foundational components. Each user story phase is designed to be independently testable to allow for continuous integration and feedback.

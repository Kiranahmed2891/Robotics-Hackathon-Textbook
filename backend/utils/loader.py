# backend/utils/loader.py

from pathlib import Path
from typing import List, Dict, Any

# Assuming textbook/docs is one level up from the backend directory
DOCS_PATH = Path(__file__).parent.parent.parent / "textbook" / "docs"

def load_markdown_documents() -> List[Dict[str, Any]]:
    """
    Scans the DOCS_PATH directory and loads all Markdown (.md) files.
    Returns a list of documents, where each document is raw text content 
    with associated metadata (source path).
    """
    documents = []
    
    # Recursively find all .md files in the docs directory
    for file_path in DOCS_PATH.glob("**/*.md"):
        try:
            # Read the entire file content
            content = file_path.read_text(encoding="utf-8")
            
            # Create a basic document structure
            document = {
                "content": content,
                "metadata": {
                    "source_path": str(file_path.relative_to(DOCS_PATH.parent.parent)),
                    # Extract the title/chapter name from the file path for better reference
                    "chapter_name": file_path.stem.replace('-', ' ').title(),
                }
            }
            documents.append(document)
            print(f"Loaded: {document['metadata']['source_path']}")

        except Exception as e:
            print(f"Error loading file {file_path}: {e}")

    return documents

# Example Test (Optional: You can run this file manually to see the output)
# if __name__ == "__main__":
#     loaded_docs = load_markdown_documents()
#     print(f"\nTotal documents loaded: {len(loaded_docs)}")
#     if loaded_docs:
#         print(f"First document content sample: {loaded_docs[0]['content'][:100]}...")
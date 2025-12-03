# backend/utils/chunker.py

from typing import List, Dict, Any
from langchain_text_splitters import RecursiveCharacterTextSplitter

# Configuration for chunking
CHUNK_SIZE = 1000
CHUNK_OVERLAP = 100

def split_documents_into_chunks(documents: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """
    Splits raw documents into smaller chunks using a text splitter optimized for Markdown.
    Inherits metadata from the parent document.
    """
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=CHUNK_SIZE,
        chunk_overlap=CHUNK_OVERLAP,
        # Set of separators optimized for Markdown/Code structure
        separators=[
            "\n\n",
            "\n",
            " ",
            "",
        ],
        length_function=len,
        is_separator_regex=False,
    )

    all_chunks = []
    
    for doc in documents:
        raw_content = doc["content"]
        base_metadata = doc["metadata"]

        # Split the text
        texts = text_splitter.split_text(raw_content)

        # Create a chunk dictionary for each split text
        for i, text in enumerate(texts):
            chunk_metadata = base_metadata.copy()
            chunk_metadata["chunk_id"] = f"{base_metadata.get('chapter_name', 'unknown')}_{i}"
            chunk_metadata["text_length"] = len(text)
            
            chunk = {
                "text": text,
                "metadata": chunk_metadata
            }
            all_chunks.append(chunk)

    print(f"Split {len(documents)} documents into {len(all_chunks)} chunks.")
    return all_chunks

# Example Test (Optional)
# if __name__ == "__main__":
#     # This requires a dummy document structure similar to loader.py output
#     dummy_documents = [
#         {
#             "content": "# Chapter 1: Introduction\n\nThis is the first paragraph. It is long and discusses AI. " * 10,
#             "metadata": {"chapter_name": "Chapter 1", "source_path": "docs/ch1.md"}
#         }
#     ]
#     chunks = split_documents_into_chunks(dummy_documents)
#     # print(chunks)
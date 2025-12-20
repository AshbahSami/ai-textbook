import argparse
import sys
from ..services.ingestion import DocumentationIngestionService
from ..services.retrieval import RAGRetrievalService


def main():
    parser = argparse.ArgumentParser(description="Documentation Ingestion Tool for RAG Chatbot")
    parser.add_argument(
        "source_type",
        choices=["url", "directory", "file"],
        help="Type of source to ingest (url, directory, or file)"
    )
    parser.add_argument(
        "source_path",
        help="Path or URL to the documentation source"
    )
    parser.add_argument(
        "--collection-name",
        default="documentation_embeddings",
        help="Name of the Qdrant collection to store embeddings (default: documentation_embeddings)"
    )
    
    args = parser.parse_args()
    
    ingestion_service = DocumentationIngestionService()
    retrieval_service = RAGRetrievalService()
    
    try:
        print(f"Starting ingestion from {args.source_type}: {args.source_path}")
        
        docs = []
        if args.source_type == "url":
            docs = ingestion_service.ingest_from_url(args.source_path)
        elif args.source_type == "directory":
            docs = ingestion_service.ingest_from_directory(args.source_path)
        elif args.source_type == "file":
            # For a single file, we'll treat it as a directory with one file
            import os
            directory = os.path.dirname(args.source_path)
            docs = ingestion_service.ingest_from_directory(directory)
            # Filter to only include the specific file
            filename = os.path.basename(args.source_path)
            docs = [doc for doc in docs if filename in doc.url or doc.title == filename]
        
        print(f"Found {len(docs)} documents to process")
        
        # Process each document
        documents_processed = 0
        for i, doc in enumerate(docs):
            print(f"Processing ({i+1}/{len(docs)}): {doc.title}")
            
            # Store the document embeddings in Qdrant
            success = retrieval_service.store_document_embeddings(doc)
            if success:
                documents_processed += 1
                print(f"  - Stored embeddings successfully")
            else:
                print(f"  - Failed to store embeddings")
        
        print(f"\nIngestion completed! Processed {documents_processed} out of {len(docs)} documents.")
        
    except Exception as e:
        print(f"Error during ingestion: {str(e)}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
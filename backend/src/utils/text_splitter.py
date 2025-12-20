import re
from typing import List
import tiktoken


class RecursiveCharacterTextSplitter:
    """
    Text splitter that recursively splits text by different characters until chunks are small enough
    """
    
    def __init__(self, chunk_size: int = 1000, chunk_overlap: int = 200, separators: List[str] = None):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        
        if separators is None:
            # Default separators in order of preference
            self.separators = ["\n\n", "\n", " ", ""]
        else:
            self.separators = separators
        
        # Initialize tokenizer for accurate token counting
        self.tokenizer = tiktoken.get_encoding("cl100k_base")
    
    def split_text(self, text: str) -> List[str]:
        """
        Split the input text into chunks
        """
        return self._split_text(text, self.separators)
    
    def _split_text(self, text: str, separators: List[str]) -> List[str]:
        """
        Recursively split text using the provided separators
        """
        # First, try to split with the first separator
        separator = separators[0]
        other_separators = separators[1:]
        
        # Split the text
        if separator == "":
            # If separator is empty, split by characters
            splits = list(text)
        else:
            # Split by the separator, keeping the separator
            splits = re.split(f'({re.escape(separator)})', text)
            # Rejoin the separator with the following text
            new_splits = []
            for i in range(0, len(splits), 2):
                if i + 1 < len(splits):
                    new_splits.append(splits[i] + splits[i + 1])
                else:
                    new_splits.append(splits[i])
            splits = new_splits
        
        # Filter out empty strings
        splits = [s for s in splits if s]
        
        # If this is the last separator or we have small enough chunks, process them
        if not other_separators:
            return self._merge_splits(splits, separator)
        else:
            # Otherwise, recursively split each chunk with the next separator
            final_splits = []
            for split in splits:
                # Check if the split is small enough
                if self._get_token_count(split) < self.chunk_size:
                    final_splits.append(split)
                else:
                    # Recursively split with the next separator
                    sub_splits = self._split_text(split, other_separators)
                    final_splits.extend(sub_splits)
            return final_splits
    
    def _merge_splits(self, splits: List[str], separator: str) -> List[str]:
        """
        Merge splits that are smaller than the chunk size
        """
        docs = []
        current_doc: List[str] = []
        total = 0
        
        for d in splits:
            _len = self._get_token_count(d)
            if total + _len >= self.chunk_size:
                if total > self.chunk_size:
                    print(f"Warning: A single chunk is longer than chunk_size. Consider increasing chunk_size or preprocessing the text.")
                
                if current_doc:
                    # Join the current doc and add it to docs
                    doc = separator.join(current_doc).strip()
                    if doc:
                        docs.append(doc)
                    
                    # Start a new doc with overlap
                    if len(current_doc) > 0:
                        # Add overlap by including some previous chunks
                        overlap_chunks = []
                        overlap_count = 0
                        
                        # Add chunks from the end until we reach overlap size
                        for i in range(len(current_doc) - 1, -1, -1):
                            chunk_len = self._get_token_count(current_doc[i])
                            if overlap_count + chunk_len <= self.chunk_overlap:
                                overlap_chunks.insert(0, current_doc[i])
                                overlap_count += chunk_len
                            else:
                                break
                        
                        current_doc = overlap_chunks
                        total = overlap_count
                    else:
                        current_doc = []
                        total = 0
                    
                    # Add the current split
                    current_doc.append(d)
                    total += _len
                else:
                    # Current doc is empty, but the split is too big
                    # So we add it as a new document
                    doc = d.strip()
                    if doc:
                        docs.append(doc)
                    current_doc = []
                    total = 0
            else:
                current_doc.append(d)
                total += _len
        
        # Add the last doc
        doc = separator.join(current_doc).strip()
        if doc:
            docs.append(doc)
        
        return docs
    
    def _get_token_count(self, text: str) -> int:
        """
        Get the number of tokens in the text
        """
        return len(self.tokenizer.encode(text))


def split_document_content(content: str, chunk_size: int = 1000, chunk_overlap: int = 200) -> List[str]:
    """
    Convenience function to split document content
    """
    splitter = RecursiveCharacterTextSplitter(
        chunk_size=chunk_size,
        chunk_overlap=chunk_overlap
    )
    return splitter.split_text(content)
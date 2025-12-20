from typing import Any, Dict
from ...services.retrieval import RAGRetrievalService


class RetrieverTool:
    """
    Tool for the agent to retrieve relevant documentation
    """
    
    def __init__(self):
        self.retrieval_service = RAGRetrievalService()
        self.name = "documentation_retriever"
        self.description = "Retrieve relevant documentation content based on user queries"
    
    def get_tool_definition(self):
        """
        Return the tool definition in the format expected by the agent
        """
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The query to search for in documentation"
                        }
                    },
                    "required": ["query"]
                }
            }
        }
    
    def execute(self, query: str) -> Dict[str, Any]:
        """
        Execute the tool with the given query
        """
        try:
            # Retrieve relevant documents
            relevant_docs = self.retrieval_service.retrieve_relevant_documents(query)
            
            # Return the relevant content
            return {
                "query": query,
                "results": [
                    {
                        "title": doc.get("title", ""),
                        "url": doc.get("url", ""),
                        "content": doc.get("content", "")[:500] + "..." if len(doc.get("content", "")) > 500 else doc.get("content", ""),
                        "score": doc.get("score", 0.0)
                    }
                    for doc in relevant_docs
                ]
            }
        except Exception as e:
            return {
                "error": f"Error retrieving documentation: {str(e)}"
            }
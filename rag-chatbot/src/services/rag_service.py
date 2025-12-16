"""
RAG service with contextual query processing
"""
import logging
import time
import google.generativeai as genai
from typing import List, Optional
from ..models.request_models import ContextualQuery, QueryResult
from ..services.qdrant_service import QdrantService
from ..services.context_consistency_service import ContextConsistencyService
from ..config.settings import settings


class RAGService:
    """
    Service for handling Retrieval-Augmented Generation queries
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.qdrant_service = QdrantService()
        self.context_consistency_service = ContextConsistencyService()

        # Initialize Gemini client
        genai.configure(api_key=settings.gemini_api_key)
        self.gemini_model = genai.GenerativeModel('gemini-2.5-pro')

    def process_contextual_query(
        self,
        session_id: str,
        selected_text: str,
        full_page_url: str,
        surrounding_context: str
    ) -> QueryResult:
        """
        Process a contextual query using RAG approach
        """
        start_time = time.time()

        try:
            # Prepare the contextual query
            contextual_query = ContextualQuery(
                session_id=session_id,
                selected_text=selected_text,
                full_page_url=full_page_url,
                primary_context=surrounding_context
            )

            # Optionally retrieve supplemental context from Qdrant
            supplemental_context = self._retrieve_supplemental_context(selected_text)
            supplemental_context_used = len(supplemental_context) > 0

            if supplemental_context_used:
                contextual_query.supplemental_context = supplemental_context

            # Construct the final prompt using both primary and supplemental context
            final_prompt = self._construct_prompt(
                contextual_query.selected_text,
                contextual_query.primary_context,
                contextual_query.supplemental_context
            )

            # Generate explanation using Gemini
            explanation = self._generate_explanation(final_prompt)

            # Evaluate context grounding quality
            context_quality_score = self._evaluate_context_quality(
                selected_text,
                surrounding_context,
                supplemental_context,
                explanation
            )

            # Calculate processing time
            processing_time_ms = int((time.time() - start_time) * 1000)

            # Return the result
            result = QueryResult(
                explanation=explanation,
                supplemental_context_used=supplemental_context_used,
                context_quality_score=context_quality_score,
                processing_time_ms=processing_time_ms
            )

            self.logger.info(f"Processed contextual query in {processing_time_ms}ms, "
                           f"supplemental context used: {supplemental_context_used}, "
                           f"context quality score: {context_quality_score}")

            return result
        except Exception as e:
            processing_time_ms = int((time.time() - start_time) * 1000)
            self.logger.error(f"Error processing contextual query: {str(e)}")
            raise e

    def _evaluate_context_quality(
        self,
        selected_text: str,
        primary_context: str,
        supplemental_context: List[str],
        explanation: str
    ) -> float:
        """
        Evaluate the quality of the context used to generate the explanation
        Returns a score between 0 and 1, where 1 is high quality
        """
        try:
            # Basic quality metrics:
            # 1. Check if selected_text appears in explanation (indicating relevance)
            relevance_score = 1.0 if selected_text.lower() in explanation.lower() else 0.5

            # 2. Check if primary_context appears in explanation (indicating proper grounding)
            context_coverage = 0.3
            for word in primary_context.lower().split()[:20]:  # Check first 20 words for efficiency
                if len(word) > 4 and word in explanation.lower():  # Only check words with >4 chars
                    context_coverage += 0.02
                    if context_coverage > 0.7:  # Cap the context coverage score
                        break

            # 3. If supplemental context was used, check if it's reflected in explanation
            supplemental_score = 0.0
            if supplemental_context:
                for context in supplemental_context:
                    if len(context) > 10 and context.lower()[:50] in explanation.lower():
                        supplemental_score = 0.2
                        break

            # Calculate overall quality score
            quality_score = min(1.0, relevance_score * 0.5 + context_coverage * 0.3 + supplemental_score * 0.2)

            return round(quality_score, 2)
        except Exception as e:
            self.logger.warning(f"Error evaluating context quality: {str(e)}")
            return 0.5  # Return neutral score on error

    def _retrieve_supplemental_context(self, query_text: str) -> List[str]:
        """
        Retrieve supplemental context from Qdrant based on the query text
        """
        try:
            # In a real implementation, we would:
            # 1. Embed the query_text using Gemini's embedding API
            # 2. Search for similar vectors in Qdrant
            # 3. Return the relevant contexts

            # For now, we'll simulate this with a placeholder
            # In a real implementation, you would use the embedding API
            # response = genai.embed_content(
            #     content=[query_text],
            #     task_type="retrieval_query",
            #     title="Contextual Query"
            # )
            # query_embedding = response['embedding'][0]

            # For this example implementation, we'll return an empty list
            # as we don't have real documents indexed in Qdrant yet
            self.logger.info(f"Retrieved supplemental context for query: {query_text[:50]}...")
            return []
        except Exception as e:
            self.logger.warning(f"Could not retrieve supplemental context: {str(e)}")
            return []

    def _construct_prompt(
        self,
        selected_text: str,
        primary_context: str,
        supplemental_context: Optional[List[str]] = None
    ) -> str:
        """
        Construct the final prompt for the LLM using primary and supplemental context
        """
        supplemental_text = ""
        if supplemental_context:
            supplemental_text = "\n\nSupplemental context from documentation:\n" + "\n".join(supplemental_context)

        prompt = (
            f"Explain the following text based on the provided context:\n\n"
            f"Selected text: {selected_text}\n\n"
            f"Primary context: {primary_context}"
            f"{supplemental_text}\n\n"
            f"Provide a clear explanation of the selected text based on the context provided. "
            f"If the context doesn't provide sufficient information, say so."
        )

        return prompt

    def _generate_explanation(self, prompt: str) -> str:
        """
        Generate explanation using the Gemini model
        """
        try:
            response = self.gemini_model.generate_content(prompt)
            explanation = response.text

            if not explanation or explanation.strip() == "":
                explanation = "I couldn't generate an explanation for the selected text based on the provided context."

            return explanation
        except Exception as e:
            self.logger.error(f"Error generating explanation: {str(e)}")
            return "An error occurred while generating the explanation. Please try again."
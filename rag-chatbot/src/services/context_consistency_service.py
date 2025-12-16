"""
Context consistency checks for quality assurance
"""
import logging
from typing import List, Dict, Any
from ..models.request_models import ContextualQuery, QueryResult


class ContextConsistencyService:
    """
    Service for checking consistency and quality of contextual explanations
    """
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
    
    def check_context_consistency(
        self, 
        query: ContextualQuery, 
        result: QueryResult
    ) -> Dict[str, Any]:
        """
        Check if the explanation is consistent with the provided context
        """
        try:
            consistency_report = {
                "is_consistent": True,
                "relevance_score": 0.0,
                "context_alignment": 0.0,
                "potential_issues": [],
                "confidence": 0.0
            }
            
            # Check if the explanation is relevant to the selected text
            relevance_score = self._check_relevance(query.selected_text, result.explanation)
            consistency_report["relevance_score"] = relevance_score
            
            # Check if the explanation aligns with the provided context
            alignment_score = self._check_context_alignment(
                query.primary_context, 
                query.supplemental_context, 
                result.explanation
            )
            consistency_report["context_alignment"] = alignment_score
            
            # Overall consistency is a combination of relevance and alignment
            overall_score = (relevance_score * 0.6 + alignment_score * 0.4)
            consistency_report["is_consistent"] = overall_score > 0.5
            consistency_report["confidence"] = overall_score
            
            # Add potential issues if scores are low
            if relevance_score < 0.3:
                consistency_report["potential_issues"].append(
                    "Low relevance: Explanation doesn't seem to address the selected text"
                )
            
            if alignment_score < 0.3:
                consistency_report["potential_issues"].append(
                    "Low alignment: Explanation doesn't align well with provided context"
                )
            
            self.logger.info(f"Context consistency check completed with score: {overall_score}")
            return consistency_report
            
        except Exception as e:
            self.logger.error(f"Error in context consistency check: {str(e)}")
            return {
                "is_consistent": False,
                "relevance_score": 0.0,
                "context_alignment": 0.0,
                "potential_issues": [f"Error during consistency check: {str(e)}"],
                "confidence": 0.0
            }
    
    def _check_relevance(self, selected_text: str, explanation: str) -> float:
        """
        Check if the explanation is relevant to the selected text
        """
        try:
            selected_lower = selected_text.lower()
            explanation_lower = explanation.lower()
            
            # Count how many significant words from selected_text appear in explanation
            selected_words = [word for word in selected_lower.split() if len(word) > 3]
            if not selected_words:
                return 0.5  # Neutral score if no words to compare
            
            matching_words = 0
            total_words = len(selected_words)
            
            for word in selected_words:
                if word in explanation_lower:
                    matching_words += 1
            
            relevance_score = matching_words / total_words
            return min(1.0, relevance_score)  # Cap at 1.0
            
        except Exception as e:
            self.logger.warning(f"Error checking relevance: {str(e)}")
            return 0.5
    
    def _check_context_alignment(
        self, 
        primary_context: str, 
        supplemental_context: List[str], 
        explanation: str
    ) -> float:
        """
        Check if the explanation aligns with the provided context
        """
        try:
            explanation_lower = explanation.lower()
            
            # Check alignment with primary context
            primary_words = [word for word in primary_context.lower().split() if len(word) > 3]
            if not primary_words:
                return 0.5  # Neutral score if no context words to compare
            
            primary_matches = 0
            for word in primary_words:
                if word in explanation_lower:
                    primary_matches += 1
            
            primary_alignment = primary_matches / len(primary_words)
            
            # Check alignment with supplemental context if available
            supplemental_alignment = 0.0
            if supplemental_context:
                total_supplemental_words = 0
                supplemental_matches = 0
                
                for context in supplemental_context:
                    supplemental_words = [word for word in context.lower().split() if len(word) > 3]
                    total_supplemental_words += len(supplemental_words)
                    
                    for word in supplemental_words:
                        if word in explanation_lower:
                            supplemental_matches += 1
                
                if total_supplemental_words > 0:
                    supplemental_alignment = supplemental_matches / total_supplemental_words
            
            # Weight primary context more heavily than supplemental
            alignment_score = (primary_alignment * 0.7) + (supplemental_alignment * 0.3)
            return min(1.0, alignment_score)  # Cap at 1.0
            
        except Exception as e:
            self.logger.warning(f"Error checking context alignment: {str(e)}")
            return 0.5
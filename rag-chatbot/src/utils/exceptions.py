"""
Error handling and logging infrastructure
"""
import logging
from fastapi import HTTPException, status
from typing import Optional
from ..config.settings import settings


# Configure logging
logging.basicConfig(
    level=logging.DEBUG if settings.debug else logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class AppException(HTTPException):
    """
    Custom application exception
    """
    def __init__(
        self,
        detail: str,
        status_code: int = status.HTTP_500_INTERNAL_SERVER_ERROR,
        error_code: Optional[str] = None
    ):
        super().__init__(status_code=status_code, detail=detail)
        self.error_code = error_code
        logger.error(f"AppException: {error_code} - {detail}")


class ValidationError(AppException):
    """
    Validation error exception
    """
    def __init__(self, detail: str, error_code: Optional[str] = "VALIDATION_ERROR"):
        super().__init__(
            detail=detail,
            status_code=status.HTTP_400_BAD_REQUEST,
            error_code=error_code
        )


class NotFoundError(AppException):
    """
    Not found error exception
    """
    def __init__(self, detail: str = "Resource not found", error_code: Optional[str] = "NOT_FOUND_ERROR"):
        super().__init__(
            detail=detail,
            status_code=status.HTTP_404_NOT_FOUND,
            error_code=error_code
        )


class DatabaseError(AppException):
    """
    Database error exception
    """
    def __init__(self, detail: str = "Database error occurred", error_code: Optional[str] = "DATABASE_ERROR"):
        super().__init__(
            detail=detail,
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            error_code=error_code
        )


class ExternalServiceError(AppException):
    """
    External service error exception (for Qdrant, Gemini, etc.)
    """
    def __init__(self, detail: str = "External service error occurred", error_code: Optional[str] = "EXTERNAL_SERVICE_ERROR"):
        super().__init__(
            detail=detail,
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            error_code=error_code
        )
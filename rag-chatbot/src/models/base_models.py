"""
Base models for the application
"""
from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class BaseResponse(BaseModel):
    """
    Base response model with standard fields
    """
    success: bool = True
    message: Optional[str] = None
    timestamp: datetime = datetime.now()


class ErrorResponse(BaseModel):
    """
    Error response model
    """
    success: bool = False
    error: str
    error_code: Optional[str] = None
    timestamp: datetime = datetime.now()
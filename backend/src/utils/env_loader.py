import os
from dotenv import load_dotenv
from typing import Optional


class EnvLoader:
    """
    Utility class for loading and accessing environment variables
    """
    
    def __init__(self, env_file: str = ".env"):
        # Load environment variables from .env file
        load_dotenv(env_file)
    
    @staticmethod
    def get(key: str, default: Optional[str] = None) -> Optional[str]:
        """
        Get an environment variable value
        """
        return os.getenv(key, default)
    
    @staticmethod
    def get_required(key: str) -> str:
        """
        Get a required environment variable value, raise exception if not found
        """
        value = os.getenv(key)
        if value is None:
            raise ValueError(f"Required environment variable '{key}' is not set")
        return value
    
    @staticmethod
    def get_bool(key: str, default: bool = False) -> bool:
        """
        Get a boolean environment variable value
        """
        value = os.getenv(key, "").lower()
        if value in ("true", "1", "yes", "on"):
            return True
        elif value in ("false", "0", "no", "off"):
            return False
        else:
            return default
    
    @staticmethod
    def get_int(key: str, default: int = 0) -> int:
        """
        Get an integer environment variable value
        """
        try:
            return int(os.getenv(key, default))
        except ValueError:
            return default
    
    @staticmethod
    def get_float(key: str, default: float = 0.0) -> float:
        """
        Get a float environment variable value
        """
        try:
            return float(os.getenv(key, default))
        except ValueError:
            return default


# Global instance for easy access
env_loader = EnvLoader()
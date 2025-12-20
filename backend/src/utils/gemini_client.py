from openai import OpenAI
import os


class GeminiClient:
    """
    Client for interacting with Google Gemini API via OpenAI-compatible endpoint
    """
    
    def __init__(self):
        self.client = OpenAI(
            base_url=os.getenv("GEMINI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/"),
            api_key=os.getenv("GEMINI_API_KEY")
        )
    
    def get_embeddings(self, texts: list[str], model: str = "text-embedding-004"):
        """
        Generate embeddings for the provided texts using Gemini API
        """
        try:
            response = self.client.embeddings.create(
                model=model,
                input=texts
            )
            return response.data
        except Exception as e:
            print(f"Error generating embeddings: {str(e)}")
            return []
    
    def generate_text(self, prompt: str, model: str = "gemini-2.0-flash", max_tokens: int = 500, temperature: float = 0.3):
        """
        Generate text using the Gemini model
        """
        try:
            response = self.client.chat.completions.create(
                model=model,
                messages=[{"role": "user", "content": prompt}],
                max_tokens=max_tokens,
                temperature=temperature
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            print(f"Error generating text: {str(e)}")
            return ""
    
    def chat_completion(self, messages: list[dict], model: str = "gemini-2.0-flash", max_tokens: int = 500, temperature: float = 0.3):
        """
        Perform a chat completion using the Gemini model
        """
        try:
            response = self.client.chat.completions.create(
                model=model,
                messages=messages,
                max_tokens=max_tokens,
                temperature=temperature
            )
            return response.choices[0].message.content.strip()
        except Exception as e:
            print(f"Error in chat completion: {str(e)}")
            return ""


# Global instance for easy access
gemini_client = GeminiClient()
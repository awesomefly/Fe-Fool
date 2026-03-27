# -*- coding: utf-8 -*-
"""
Volcano Engine Ark API Client
火山引擎方舟大模型API客户端
Based on: https://www.volcengine.com/docs/82379/1298459
"""
import json
import time
import requests
from typing import Dict, Any, Optional, List, Union
from requests.adapters import HTTPAdapter
from urllib3.util.retry import Retry

from .config import Config
from .exceptions import VolcanoAPIError, AuthenticationError, RateLimitError


class VolcanoArkClient:
    """Client for Volcano Engine Ark API (大模型服务平台)"""

    def __init__(self, api_key: Optional[str] = None, endpoint: Optional[str] = None):
        """
        Initialize Volcano Engine Ark client

        Args:
            api_key: API key for authentication
            endpoint: API endpoint URL
        """
        self.api_key = api_key or Config.VOLCANO_API_KEY
        self.endpoint = endpoint or Config.VOLCANO_ENDPOINT

        if not self.api_key:
            raise AuthenticationError("API key is required")

        # Setup session with retry strategy
        self.session = requests.Session()
        retry_strategy = Retry(
            total=Config.MAX_RETRIES,
            backoff_factor=1,
            status_forcelist=[429, 500, 502, 503, 504],
        )
        adapter = HTTPAdapter(max_retries=retry_strategy)
        self.session.mount("http://", adapter)
        self.session.mount("https://", adapter)

    def _prepare_headers(self) -> Dict[str, str]:
        """Prepare request headers"""
        return {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}',
            'Accept': 'application/json',
        }

    def chat_completions(
        self,
        model: str,
        messages: List[Dict[str, str]],
        temperature: float = 0.7,
        max_tokens: int = 1000,
        top_p: float = 1.0,
        n: int = 1,
        stream: bool = False,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Chat completions API - 对话生成

        Args:
            model: Model endpoint ID (e.g., "ep-20250904201713-k69v5")
            messages: List of conversation messages
            temperature: Sampling temperature (0-2)
            max_tokens: Maximum tokens to generate
            top_p: Nucleus sampling parameter
            n: Number of completions to generate
            stream: Whether to stream responses
            **kwargs: Additional parameters

        Returns:
            API response
        """
        url = f"{self.endpoint}"

        payload = {
            "model": model,
            "messages": messages,
            "temperature": temperature,
            "max_tokens": max_tokens,
            "top_p": top_p,
            "n": n,
            "stream": stream,
        }

        # Add any additional parameters
        payload.update(kwargs)

        headers = self._prepare_headers()

        try:
            response = self.session.post(
                url,
                headers=headers,
                json=payload,
                timeout=Config.REQUEST_TIMEOUT
            )

            # Handle rate limiting
            if response.status_code == 429:
                raise RateLimitError("API rate limit exceeded")

            # Handle authentication errors
            if response.status_code == 401:
                raise AuthenticationError("Invalid API key")

            # Handle other errors
            response.raise_for_status()

            return response.json()

        except requests.exceptions.RequestException as e:
            raise VolcanoAPIError(f"API request failed: {str(e)}")
        except json.JSONDecodeError as e:
            raise VolcanoAPIError(f"Failed to parse JSON response: {str(e)}")

    def multimodal_chat(
        self,
        model: str,
        messages: List[Dict[str, Any]],
        temperature: float = 0.7,
        max_tokens: int = 1000,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Multimodal chat API for vision-language models
        多模态对话API - 支持图片输入

        Args:
            model: Model endpoint ID
            messages: List of messages with potential image content
            temperature: Sampling temperature
            max_tokens: Maximum tokens to generate
            **kwargs: Additional parameters

        Returns:
            API response with generated content
        """
        return self.chat_completions(
            model=model,
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens,
            **kwargs
        )

    def embeddings(
        self,
        model: str,
        input: Union[str, List[str]],
        encoding_format: str = "float",
        **kwargs
    ) -> Dict[str, Any]:
        """
        Embeddings API - 文本向量化

        Args:
            model: Embedding model endpoint ID
            input: Text or list of texts to embed
            encoding_format: Format of embeddings ("float" or "base64")
            **kwargs: Additional parameters

        Returns:
            Embeddings response
        """
        url = f"{self.endpoint.replace('chat/completions', 'embeddings')}"

        payload = {
            "model": model,
            "input": input,
            "encoding_format": encoding_format,
        }
        payload.update(kwargs)

        headers = self._prepare_headers()

        try:
            response = self.session.post(
                url,
                headers=headers,
                json=payload,
                timeout=Config.REQUEST_TIMEOUT
            )

            response.raise_for_status()
            return response.json()

        except requests.exceptions.RequestException as e:
            raise VolcanoAPIError(f"Embeddings API request failed: {str(e)}")

    def list_models(self) -> Dict[str, Any]:
        """
        List available models - 获取模型列表

        Returns:
            List of available models
        """
        url = f"{self.endpoint.rsplit('/', 1)[0]}/models"
        headers = self._prepare_headers()

        try:
            response = self.session.get(
                url,
                headers=headers,
                timeout=Config.REQUEST_TIMEOUT
            )

            response.raise_for_status()
            return response.json()

        except requests.exceptions.RequestException as e:
            raise VolcanoAPIError(f"List models request failed: {str(e)}")

    def create_message_with_image(
        self,
        role: str,
        content: str,
        image_url: Optional[str] = None,
        image_base64: Optional[str] = None,
        image_type: str = "image/jpeg"
    ) -> Dict[str, Any]:
        """
        Helper method to create a message with image content

        Args:
            role: Message role ("user", "assistant", "system")
            content: Text content
            image_url: URL of image (optional)
            image_base64: Base64 encoded image (optional)
            image_type: MIME type of image

        Returns:
            Formatted message
        """
        message = {
            "role": role,
            "content": content
        }

        if image_url or image_base64:
            # For multimodal models, format content as list
            message["content"] = [
                {
                    "type": "text",
                    "text": content
                }
            ]

            if image_url:
                message["content"].append({
                    "type": "image_url",
                    "image_url": {
                        "url": image_url
                    }
                })
            elif image_base64:
                message["content"].append({
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:{image_type};base64,{image_base64}"
                    }
                })

        return message
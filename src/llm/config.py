# -*- coding: utf-8 -*-
"""
Configuration module for Volcano Engine API
"""
import os
from typing import Optional


class Config:
    """Configuration class for Volcano Engine API settings"""

    # Volcano Engine Ark API Configuration (大模型服务平台)
    VOLCANO_API_KEY: Optional[str] = os.getenv("VOLCANO_API_KEY")
    VOLCANO_ENDPOINT: str = os.getenv(
        "VOLCANO_ENDPOINT", "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
    )

    # Default model settings
    DEFAULT_MODEL: str = ""

    # Request settings
    REQUEST_TIMEOUT: int = 120
    MAX_RETRIES: int = 3

    # Image processing settings
    MAX_IMAGE_SIZE: int = 50 * 1024 * 1024  # 50MB
    SUPPORTED_IMAGE_FORMATS: list = ["jpg", "jpeg", "png", "bmp", "webp"]

    @classmethod
    def validate_config(cls) -> bool:
        """Validate that required configuration is present"""
        if not cls.VOLCANO_API_KEY:
            raise ValueError("VOLCANO_API_KEY environment variable is required")
        # Secret key is not required for Ark API
        return True

    @classmethod
    def set_key_and_endpoint(cls, api_key: str, endpoint: str):
        """Set API keys programmatically"""
        cls.VOLCANO_API_KEY = api_key
        cls.VOLCANO_ENDPOINT = endpoint

    @classmethod
    def set_models(cls, default_model: str):
        """Set default models programmatically"""
        cls.DEFAULT_MODEL = default_model

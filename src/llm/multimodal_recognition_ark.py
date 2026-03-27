# -*- coding: utf-8 -*-
"""
Multimodal Recognition Service using Volcano Engine Ark API
基于火山引擎方舟大模型的多模态识别服务
"""
import base64
import imghdr
import os
from io import BytesIO
from typing import Dict, Any, Optional, Union, List
from PIL import Image
import cv2
import numpy as np

from .volcano_ark_client import VolcanoArkClient
from .config import Config
from .exceptions import ImageProcessingError, InvalidRequestError


class MultimodalRecognizerArk:
    """Service for multimodal image recognition using Volcano Engine Ark API"""

    def __init__(self, api_key: Optional[str] = None, endpoint: Optional[str] = None):
        """
        Initialize multimodal recognizer

        Args:
            api_key: Volcano Engine API key
            endpoint: API endpoint URL
        """
        self.client = VolcanoArkClient(api_key, endpoint)

    def _validate_image(self, image_path: str) -> bool:
        """Validate image file"""
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image file not found: {image_path}")

        file_size = os.path.getsize(image_path)
        if file_size > Config.MAX_IMAGE_SIZE:
            raise InvalidRequestError(
                f"Image size exceeds maximum allowed size of {Config.MAX_IMAGE_SIZE} bytes"
            )

        image_type = imghdr.what(image_path)
        if image_type not in Config.SUPPORTED_IMAGE_FORMATS:
            raise InvalidRequestError(
                f"Unsupported image format. Supported formats: {Config.SUPPORTED_IMAGE_FORMATS}"
            )

        return True

    def _preprocess_image(
        self, image: Union[str, np.ndarray, Image.Image], max_size: tuple = (4069, 4069)
    ) -> str:
        """
        Preprocess image for API submission

        Args:
            image: Image path, numpy array, or PIL Image
            max_size: Maximum dimensions (width, height)

        Returns:
            Base64 encoded image string
        """
        try:
            # Load image based on input type
            if isinstance(image, str):
                self._validate_image(image)
                pil_image = Image.open(image)
            elif isinstance(image, np.ndarray):
                # Convert OpenCV image (BGR) to PIL (RGB)
                if len(image.shape) == 3:
                    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                pil_image = Image.fromarray(image)
            elif isinstance(image, Image.Image):
                pil_image = image
            else:
                raise InvalidRequestError("Unsupported image type")

            # Resize if necessary while maintaining aspect ratio
            pil_image.thumbnail(max_size, Image.Resampling.LANCZOS)

            # Convert to RGB if necessary
            if pil_image.mode != 'RGB':
                pil_image = pil_image.convert('RGB')

            # Save to bytes buffer
            buffer = BytesIO()
            pil_image.save(buffer, format='JPEG', quality=95)
            image_bytes = buffer.getvalue()

            # Encode to base64
            base64_string = base64.b64encode(image_bytes).decode('utf-8')

            return base64_string

        except Exception as e:
            raise ImageProcessingError(f"Failed to preprocess image: {str(e)}")

    def analyze_image(
        self, image: Union[str, np.ndarray, Image.Image], prompt: str, model: str = None
    ) -> Dict[str, Any]:
        """
        Analyze image with multimodal model

        Args:
            image: Image path, numpy array, or PIL Image
            prompt: Text prompt for analysis
            model: Model endpoint ID (uses default if not specified)

        Returns:
            Analysis results
        """
        # Preprocess image
        image_base64 = self._preprocess_image(image)

        # Create message with image
        messages = [
            self.client.create_message_with_image(
                role="user",
                content=prompt,
                image_base64=image_base64,
                image_type="image/jpeg",
            )
        ]

        # Send request
        result = self.client.multimodal_chat(
            model=model or Config.DEFAULT_MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=1000,
        )

        return result

    def detect_chess_pieces(
        self, image: Union[str, np.ndarray, Image.Image], model: str = None
    ) -> List[Dict[str, Any]]:
        """
        Detect and classify chess pieces in image

        Args:
            image: Image path, numpy array, or PIL Image

        Returns:
            List of detected pieces with positions and types
        """
        import os

        prompt_file_path = os.path.join(
            os.path.dirname(__file__), "prompt", "chess_analysis.md"
        )
        with open(prompt_file_path, "r", encoding="utf-8") as f:
            prompt = f.read()

        result = self.analyze_image(image, prompt, model=model)

        # Extract pieces from response
        content = []
        if "choices" in result and result["choices"]:
            str_content = result["choices"][0].get("message", {}).get("content", "")
            try:
                # Try to parse JSON content
                import json

                content = json.loads(str_content)
            except:
                # If not valid JSON, return raw content
                print(
                    f"Response content is not valid JSON, returning raw content: {str_content}"
                )

        return content

    def simple_chat(
        self, message: str, model: str = None, system_prompt: str = None
    ) -> str:
        """
        Simple text-only chat

        Args:
            message: User message
            model: Model endpoint ID
            system_prompt: Optional system prompt

        Returns:
            Assistant response
        """
        messages = []

        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})

        messages.append({"role": "user", "content": message})

        result = self.client.chat_completions(
            model=model or Config.DEFAULT_MULTIMODAL_MODEL,
            messages=messages,
            temperature=0.7,
            max_tokens=1000,
        )

        # Extract response
        if "choices" in result and result["choices"]:
            return result["choices"][0].get("message", {}).get("content", "")

        return ""

# -*- coding: utf-8 -*-
"""
LLM Module for Volcano Engine Multimodal Recognition
火山引擎多模态识别模型模块
"""

# New Ark API client (recommended)
from .volcano_ark_client import VolcanoArkClient
from .multimodal_recognition_ark import MultimodalRecognizerArk

from .config import Config

__version__ = "2.0.0"
__all__ = [
    "VolcanoEngineClient",
    "MultimodalRecognizer",  # Legacy
    "VolcanoArkClient",
    "MultimodalRecognizerArk",  # New Ark API
    "Config",
]

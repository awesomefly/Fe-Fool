# -*- coding: utf-8 -*-
"""
Custom exceptions for Volcano Engine API
"""


class VolcanoAPIError(Exception):
    """Base exception for Volcano Engine API errors"""
    pass


class AuthenticationError(VolcanoAPIError):
    """Raised when authentication fails"""
    pass


class RateLimitError(VolcanoAPIError):
    """Raised when API rate limit is exceeded"""
    pass


class InvalidRequestError(VolcanoAPIError):
    """Raised when request is invalid"""
    pass


class ModelNotFoundError(VolcanoAPIError):
    """Raised when specified model is not found"""
    pass


class ImageProcessingError(VolcanoAPIError):
    """Raised when image processing fails"""
    pass
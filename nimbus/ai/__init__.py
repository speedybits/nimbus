"""
AI module for Nimbus - Ollama integration for intelligent exploration.

This module provides:
- OllamaClient: REST client for Ollama LLM
- LidarDescriber: Convert LIDAR data to natural language
- ExplorationMemory: Track and persist explored areas
"""

from nimbus.ai.ollama import OllamaClient, OllamaConfig
from nimbus.ai.describer import LidarDescriber
from nimbus.ai.memory import ExplorationMemory

__all__ = [
    "OllamaClient",
    "OllamaConfig",
    "LidarDescriber",
    "ExplorationMemory",
]

"""
Ollama REST client for Nimbus AI exploration.

Simple, synchronous client following ROSIE's proven pattern.
Makes HTTP requests to local Ollama instance for waypoint decisions.
"""

import json
import os
import re
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable
import requests


@dataclass(frozen=True)
class OllamaConfig:
    """Ollama client configuration."""

    model: str = field(default_factory=lambda: os.getenv("NIMBUS_OLLAMA_MODEL", "llama3.1:8b"))
    url: str = field(default_factory=lambda: os.getenv("NIMBUS_OLLAMA_URL", "http://localhost:11434"))
    temperature: float = 0.3  # Low for consistent decisions
    max_tokens: int = 200
    timeout: float = 30.0  # seconds


@dataclass
class WaypointDecision:
    """Result of an Ollama waypoint decision."""

    x: float  # Relative X offset from current position (meters)
    y: float  # Relative Y offset from current position (meters)
    reason: str  # Explanation for the decision
    success: bool = True
    error: Optional[str] = None

    @classmethod
    def failed(cls, error: str) -> "WaypointDecision":
        """Create a failed decision."""
        return cls(x=0.0, y=0.0, reason="", success=False, error=error)


# Exploration prompt template
EXPLORATION_PROMPT = """You are controlling a small wheeled robot exploring an indoor space.

CURRENT STATE:
{description}

EXPLORATION MEMORY:
- Areas explored: {explored_summary}
- Recent path: {recent_path}
- Previous decisions: {recent_decisions}

GOAL: Explore unknown areas while avoiding revisiting the same spots.

GUIDELINES:
- Prefer open spaces over tight corridors
- Move toward unexplored directions
- Avoid backtracking unless necessary
- Stay within safe operating range (1-3 meters per waypoint)
- If stuck or surrounded, suggest small rotation or backup

Respond with JSON only, no other text:
{{"x": <meters_relative_forward_positive>, "y": <meters_relative_left_positive>, "reason": "<brief explanation>"}}
"""


class OllamaClient:
    """
    REST client for Ollama LLM.

    Makes synchronous HTTP requests to local Ollama instance.
    Designed to be called from a background thread to avoid
    blocking the main control loop.

    Usage:
        client = OllamaClient()
        decision = client.decide_waypoint(description, memory_summary)
        if decision.success:
            target_x = current_x + decision.x
            target_y = current_y + decision.y
    """

    def __init__(self, config: Optional[OllamaConfig] = None):
        self.config = config or OllamaConfig()
        self._api_url = f"{self.config.url}/api/generate"
        self._health_url = f"{self.config.url}/api/tags"

    def is_available(self) -> bool:
        """Check if Ollama is running and accessible."""
        try:
            response = requests.get(self._health_url, timeout=5.0)
            return response.status_code == 200
        except requests.RequestException:
            return False

    def decide_waypoint(
        self,
        description: str,
        explored_summary: str = "None yet",
        recent_path: str = "Just started",
        recent_decisions: str = "None yet",
    ) -> WaypointDecision:
        """
        Ask Ollama for the next waypoint to explore.

        Args:
            description: Natural language description of current environment
            explored_summary: Summary of areas already explored
            recent_path: Recent positions visited
            recent_decisions: Last few Ollama decisions

        Returns:
            WaypointDecision with relative x, y offsets and reason
        """
        prompt = EXPLORATION_PROMPT.format(
            description=description,
            explored_summary=explored_summary,
            recent_path=recent_path,
            recent_decisions=recent_decisions,
        )

        try:
            response = requests.post(
                self._api_url,
                json={
                    "model": self.config.model,
                    "prompt": prompt,
                    "temperature": self.config.temperature,
                    "num_predict": self.config.max_tokens,
                    "stream": False,
                },
                timeout=self.config.timeout,
            )

            if response.status_code != 200:
                return WaypointDecision.failed(
                    f"Ollama returned status {response.status_code}"
                )

            result = response.json()
            ollama_text = result.get("response", "").strip()

            return self._parse_response(ollama_text)

        except requests.Timeout:
            return WaypointDecision.failed("Ollama request timed out")
        except requests.RequestException as e:
            return WaypointDecision.failed(f"Ollama request failed: {e}")
        except Exception as e:
            return WaypointDecision.failed(f"Unexpected error: {e}")

    def _parse_response(self, text: str) -> WaypointDecision:
        """Parse Ollama's JSON response into a WaypointDecision."""
        try:
            # Extract JSON from response (may have extra text)
            json_match = re.search(r"\{[^}]+\}", text, re.DOTALL)
            if not json_match:
                return WaypointDecision.failed(f"No JSON found in response: {text[:100]}")

            data = json.loads(json_match.group(0))

            x = float(data.get("x", 0))
            y = float(data.get("y", 0))
            reason = str(data.get("reason", "No reason given"))

            # Validate reasonable range
            if abs(x) > 5.0 or abs(y) > 5.0:
                return WaypointDecision.failed(
                    f"Waypoint too far: ({x}, {y}). Max 5m per step."
                )

            return WaypointDecision(x=x, y=y, reason=reason)

        except json.JSONDecodeError as e:
            return WaypointDecision.failed(f"Invalid JSON: {e}")
        except (ValueError, TypeError) as e:
            return WaypointDecision.failed(f"Invalid waypoint values: {e}")


class AsyncOllamaClient:
    """
    Async wrapper for OllamaClient.

    Runs Ollama requests in a background thread to avoid blocking
    the main control loop. Results are delivered via callback.

    Usage:
        client = AsyncOllamaClient()
        client.request_waypoint(description, memory, callback=on_decision)
        # ... later ...
        if client.has_result():
            decision = client.get_result()
    """

    def __init__(self, config: Optional[OllamaConfig] = None):
        self._sync_client = OllamaClient(config)
        self._lock = threading.Lock()
        self._pending_request = False
        self._result: Optional[WaypointDecision] = None
        self._thread: Optional[threading.Thread] = None

    def is_available(self) -> bool:
        """Check if Ollama is running."""
        return self._sync_client.is_available()

    def request_waypoint(
        self,
        description: str,
        explored_summary: str = "None yet",
        recent_path: str = "Just started",
        recent_decisions: str = "None yet",
        callback: Optional[Callable[[WaypointDecision], None]] = None,
    ) -> bool:
        """
        Request a waypoint decision asynchronously.

        Returns False if a request is already pending.
        """
        with self._lock:
            if self._pending_request:
                return False
            self._pending_request = True
            self._result = None

        def worker():
            result = self._sync_client.decide_waypoint(
                description, explored_summary, recent_path, recent_decisions
            )
            with self._lock:
                self._result = result
                self._pending_request = False
            if callback:
                callback(result)

        self._thread = threading.Thread(target=worker, daemon=True)
        self._thread.start()
        return True

    def is_pending(self) -> bool:
        """Check if a request is in progress."""
        with self._lock:
            return self._pending_request

    def has_result(self) -> bool:
        """Check if a result is available."""
        with self._lock:
            return self._result is not None

    def get_result(self) -> Optional[WaypointDecision]:
        """Get the result and clear it."""
        with self._lock:
            result = self._result
            self._result = None
            return result

    def cancel(self) -> None:
        """Cancel any pending request (result will be ignored)."""
        with self._lock:
            self._pending_request = False
            self._result = None

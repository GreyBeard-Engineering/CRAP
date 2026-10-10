"""Helpers for writing generated files."""
import os


def ensure_parent(path: str) -> str:
    """Create path's directory if needed; returns path."""
    if os.path.dirname(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
    return path

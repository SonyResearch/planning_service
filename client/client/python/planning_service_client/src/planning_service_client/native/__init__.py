"""
Native (C++/pybind11) bindings for planning_service_client.

This package provides a stable Python import surface organized into submodules:

    planning_service_client.native.types
    planning_service_client.native.visualizer
"""

from __future__ import annotations

__all__ = [
    "types",  # type: ignore
    "visualizer",  # type: ignore
]


def __getattr__(name: str):
    if name in __all__:
        import importlib

        mod = importlib.import_module(f"planning_service_client.native.{name}")
        globals()[name] = mod
        return mod
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

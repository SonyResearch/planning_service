"""
Single point of truth for loading the pybind11 extension module.

All native submodules (types.py, visualizer.py, …) should import from here rather than each repeating the sys.modules
guard.

from planning_service_client.native._ext import native_ext
"""

from __future__ import annotations

import sys

_EXTENSION_MODULE = "planning_service_client.native._planning_service_client"


def __getattr__(name: str):
    if name == "native_ext":
        if _EXTENSION_MODULE in sys.modules:
            mod = sys.modules[_EXTENSION_MODULE]
        else:
            import importlib

            try:
                mod = importlib.import_module(_EXTENSION_MODULE)
            except ModuleNotFoundError:
                from . import _planning_service_client as mod  # type: ignore[assignment]
        globals()["native_ext"] = mod
        return mod
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

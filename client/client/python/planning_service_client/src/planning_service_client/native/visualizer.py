"""
Native visualizer client bindings.

This submodule provides a clearer import location for the native (C++/pybind11)
visualizer client:

    from planning_service_client.native.visualizer import VisualizerClient

The implementation is provided by the compiled extension module.
"""

from __future__ import annotations

from planning_service_client.native._ext import native_ext

_visualizer = native_ext.visualizer

VisualizerClient = _visualizer.VisualizerClient

__all__ = ["VisualizerClient"]

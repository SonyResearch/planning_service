# Package info
name = "planning_service_client"
__version__ = "0.0.1"

# Prefer native (C++/pybind11) bindings when available.
try:
    from planning_service_client.native import *  # noqa: F403 F401
except (ImportError, ModuleNotFoundError, OSError) as e:
    # Keep package importable even when the native extension isn't built.
    import warnings

    warnings.warn(f"Native pybind11 extension unavailable: {e}", ImportWarning)

import sys

import pytest

"""Wrapper which will invoke pytest on a list of files at argv, i.e. as in `bazel test ...`."""

if __name__ == "__main__":
    sys.exit(pytest.main(["-s"] + sys.argv[1:]))

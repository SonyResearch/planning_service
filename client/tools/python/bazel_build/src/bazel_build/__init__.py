import platform
import shutil
import stat
import subprocess
import tempfile
from typing import List, Optional, Tuple
import urllib.request
from pathlib import Path

BAZELISK_VERSION = "1.25.0"

_SYSTEM_MAP = {"Linux": "linux", "Darwin": "darwin", "Windows": "windows"}
_MACHINE_MAP = {"x86_64": "amd64", "aarch64": "arm64", "arm64": "arm64"}


def bazelisk_url() -> str:
    system = _SYSTEM_MAP.get(platform.system(), platform.system().lower())
    machine = _MACHINE_MAP.get(platform.machine(), platform.machine().lower())
    name = f"bazelisk-{system}-{machine}"
    if system == "windows":
        name += ".exe"
    return f"https://github.com/bazelbuild/bazelisk/releases/download/v{BAZELISK_VERSION}/{name}"


def download_bazelisk(dest: Path) -> None:
    """
    Download the bazelisk binary for the current platform to *dest* and make it executable.
    """
    url = bazelisk_url()
    print(f"Downloading bazelisk from {url}")
    with urllib.request.urlopen(url) as response:  # noqa: S310
        dest.write_bytes(response.read())
    dest.chmod(dest.stat().st_mode | stat.S_IEXEC)


def find_workspace_root(start: Path) -> Path:
    """
    Walk upwards from *start* until a directory containing MODULE.bazel is found.
    """
    for candidate in [start, *start.parents]:
        if (candidate / "MODULE.bazel").exists():
            return candidate
    raise RuntimeError(
        f"Could not find a Bazel workspace (MODULE.bazel) searching upward from '{start}'. "
        "Use an editable install (uv pip install -e) or set SKIP_NATIVE_BUILD=1."
    )


def resolve_bazel() -> Tuple[str, Optional[str]]:
    """
    Return a path to bazel or bazelisk, downloading bazelisk into a temp dir if neither is on PATH.

    The caller is responsible for cleaning up the temp directory if one is created. Returns (bazel_executable,
    tmpdir_or_None).
    """
    bazel = shutil.which("bazel") or shutil.which("bazelisk")
    tmpdir = None
    if bazel is None:
        print("bazel/bazelisk not found on PATH, downloading bazelisk...")
        tmpdir = tempfile.mkdtemp()
        dest = Path(tmpdir) / "bazelisk"
        download_bazelisk(dest)
        bazel = str(dest)
    return (bazel, tmpdir)


def run_bazel_build(target: str, workspace_root: Path, flags: Optional[List[str]] = None) -> None:
    """
    Invoke ``bazel build`` for *target*
    """
    bazel, tmpdir = resolve_bazel()
    try:
        subprocess.run(
            [bazel, "build", *(flags or []), target],
            check=True,
            cwd=str(workspace_root),
        )
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Bazel build failed for target '{target}'.") from e
    finally:
        if tmpdir is not None:
            shutil.rmtree(tmpdir, ignore_errors=True)

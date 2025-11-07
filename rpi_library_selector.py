"""GPIO library selection helpers for Raspberry Pi boards.

This module inspects the current platform to determine which GPIO library
should be used by the project. Raspberry Pi boards up to and including the
Model 4 work with the pigpio C/Python bindings while the Model 5 switched to
lgpio. The helper can be used programmatically or from the command line to
identify the best match for the running system.
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import os
import re
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

_MODEL_PATHS = (
    Path("/sys/firmware/devicetree/base/model"),
    Path("/proc/device-tree/model"),
)
_CPUINFO_PATH = Path("/proc/cpuinfo")
_ENV_MODEL_OVERRIDE = "IR_PI_MODEL_OVERRIDE"
_ENV_FORCE_LIB = "IR_PI_FORCE_LIB"


def _clean_model_string(raw: str) -> str:
    """Normalize model strings read from device-tree files."""
    return raw.replace("\x00", "").strip()


def detect_pi_model() -> Optional[str]:
    """Return the Raspberry Pi model string if it can be detected."""
    override = os.environ.get(_ENV_MODEL_OVERRIDE)
    if override:
        return override.strip()

    for path in _MODEL_PATHS:
        try:
            model = _clean_model_string(path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            continue
        if model:
            return model

    if _CPUINFO_PATH.exists():
        model_line_prefix = "Model\t:"
        for line in _CPUINFO_PATH.read_text(encoding="utf-8").splitlines():
            if line.startswith(model_line_prefix):
                return line[len(model_line_prefix) :].strip()

    return None


_VERSION_RE = re.compile(r"Raspberry Pi[^0-9]*(?P<version>\d+)", re.IGNORECASE)
_SUPPORTED_LIBRARIES: Tuple[str, ...] = ("pigpio", "lgpio")


def parse_pi_version(model: Optional[str]) -> Optional[int]:
    """Extract the major Raspberry Pi version number from *model*."""
    if not model:
        return None

    match = _VERSION_RE.search(model)
    if match:
        try:
            return int(match.group("version"))
        except ValueError:
            return None

    return None


def _detect_available_libraries() -> List[str]:
    """Return the supported GPIO libraries that are importable on this host."""

    available: List[str] = []
    for lib in _SUPPORTED_LIBRARIES:
        try:
            spec = importlib.util.find_spec(lib)
        except (ImportError, ValueError):
            spec = None
        if spec is not None:
            available.append(lib)
    return available


def _order_candidates(version: Optional[int]) -> List[str]:
    """Return the library preference order for the detected Pi version."""

    order: List[str] = []
    if version and version >= 5:
        order.extend(["lgpio", "pigpio"])
    else:
        order.extend(["pigpio", "lgpio"])

    # Make sure any remaining supported libraries are appended to the order so
    # future additions still appear in the result.
    for lib in _SUPPORTED_LIBRARIES:
        if lib not in order:
            order.append(lib)
    return order


def _select_from_candidates(
    candidates: Iterable[str],
    *, available: Iterable[str],
    require_installed: bool,
) -> Optional[str]:
    """Return the first candidate present in *available* or the first fallback.

    When *require_installed* is ``True`` a selection is only returned if a
    candidate is installed. Otherwise the first candidate is returned even if
    none are importable. This behaviour matches the historical implementation
    which always preferred pigpio unless the Pi 5 heuristic kicked in.
    """

    available_set = {lib.lower() for lib in available}
    first_candidate: Optional[str] = None
    for name in candidates:
        name = name.lower()
        if first_candidate is None:
            first_candidate = name
        if name in available_set:
            return name

    if require_installed:
        return None
    return first_candidate


def select_gpio_library(
    *,
    model: Optional[str] = None,
    version: Optional[int] = None,
    installed_only: bool = False,
) -> str:
    """Return the preferred GPIO library name for the detected Raspberry Pi.

    Parameters
    ----------
    model:
        Override the detected Raspberry Pi model string. ``None`` triggers
        automatic detection.
    version:
        Override the Raspberry Pi major version number. ``None`` triggers
        parsing of the ``model`` string.
    installed_only:
        When ``True`` the function will raise a :class:`RuntimeError` if none of
        the supported libraries are importable. When ``False`` (the default)
        the first candidate will be returned even if it is not currently
        installed, preserving backwards compatibility with callers that expect
        to handle import errors separately.
    """

    forced = os.environ.get(_ENV_FORCE_LIB)
    if forced:
        return forced.strip().lower()

    if model is None:
        model = detect_pi_model()
    if version is None:
        version = parse_pi_version(model)

    candidates = _order_candidates(version)
    available = _detect_available_libraries()
    selected = _select_from_candidates(
        candidates,
        available=available,
        require_installed=installed_only,
    )

    if selected is None:
        raise RuntimeError(
            "No supported GPIO libraries are installed. Install pigpio or lgpio"
        )

    return selected


def load_gpio_library(
    *, model: Optional[str] = None, version: Optional[int] = None
) -> Tuple[str, object]:
    """Import and return the preferred GPIO library module.

    Returns a tuple containing the resolved library name and the imported
    module object. The import will raise :class:`ImportError` if the library is
    not installed on the current system.
    """

    lib_name = select_gpio_library(
        model=model,
        version=version,
        installed_only=True,
    )
    module = importlib.import_module(lib_name)
    return lib_name, module


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Detect the Raspberry Pi model and print the recommended GPIO library"
        )
    )
    parser.add_argument(
        "--model",
        help="Override the detected Raspberry Pi model (useful for testing).",
    )
    parser.add_argument(
        "--version",
        type=int,
        help="Explicitly provide the Raspberry Pi major version number.",
    )
    parser.add_argument(
        "--load",
        action="store_true",
        help="Attempt to import the selected library and report its location.",
    )
    return parser


def main() -> int:
    parser = _build_arg_parser()
    args = parser.parse_args()

    model = args.model
    if model is None:
        model = detect_pi_model()

    version = args.version
    if version is None:
        version = parse_pi_version(model)

    lib = select_gpio_library(model=model, version=version)
    print(lib)

    if args.load:
        try:
            lib_name, module = load_gpio_library(model=model, version=version)
        except (ImportError, RuntimeError) as exc:
            parser.error(str(exc))
        module_path = getattr(module, "__file__", None)
        if module_path:
            print(module_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

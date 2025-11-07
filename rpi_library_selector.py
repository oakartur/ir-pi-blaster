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
import os
import re
from pathlib import Path
from typing import Optional, Tuple

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


_VERSION_RE = re.compile(r"Raspberry Pi\s*(?:Model\s*)?(?P<version>\d+)", re.IGNORECASE)


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


def select_gpio_library(
    *, model: Optional[str] = None, version: Optional[int] = None
) -> str:
    """Return the preferred GPIO library name for the detected Raspberry Pi."""
    forced = os.environ.get(_ENV_FORCE_LIB)
    if forced:
        return forced.strip().lower()

    if model is None:
        model = detect_pi_model()
    if version is None:
        version = parse_pi_version(model)

    if version and version >= 5:
        return "lgpio"

    return "pigpio"


def load_gpio_library(
    *, model: Optional[str] = None, version: Optional[int] = None
) -> Tuple[str, object]:
    """Import and return the preferred GPIO library module.

    Returns a tuple containing the resolved library name and the imported
    module object. The import will raise :class:`ImportError` if the library is
    not installed on the current system.
    """

    lib_name = select_gpio_library(model=model, version=version)
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
        lib_name, module = load_gpio_library(model=model, version=version)
        module_path = getattr(module, "__file__", None)
        if module_path:
            print(module_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

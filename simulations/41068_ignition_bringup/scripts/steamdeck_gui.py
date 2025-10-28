#!/usr/bin/env python3

from pathlib import Path
import sys
import importlib.util


def _load_main_from_repo() -> None:
    # Try to load the implementation from the repo root to avoid name clashes
    repo_script = Path(__file__).resolve().parents[3] / 'steamdeck_gui.py'
    if not repo_script.exists():
        raise ImportError(f"steamdeck_gui.py not found at {repo_script}")
    spec = importlib.util.spec_from_file_location('steamdeck_gui_impl', str(repo_script))
    if spec is None or spec.loader is None:
        raise ImportError("Failed to create module spec for steamdeck_gui_impl")
    module = importlib.util.module_from_spec(spec)
    sys.modules['steamdeck_gui_impl'] = module
    spec.loader.exec_module(module)  # type: ignore[attr-defined]
    module.main()


def main():
    # First, try importing the implementation if available on PYTHONPATH
    try:
        from steamdeck_gui import main as impl_main  # type: ignore
        impl_main()
        return
    except Exception:
        pass
    # Fallback to loading from source tree
    _load_main_from_repo()


if __name__ == '__main__':
    main()



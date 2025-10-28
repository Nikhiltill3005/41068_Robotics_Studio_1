#!/usr/bin/env python3

from pathlib import Path
import sys

# Allow running whether installed or from source by falling back to repo script
try:
    from steamdeck_gui import main  # type: ignore
except Exception:
    # Fallback: try to import from repository root if not installed into PYTHONPATH
    repo_script = Path(__file__).resolve().parents[2] / 'steamdeck_gui.py'
    if repo_script.exists():
        sys.path.insert(0, str(repo_script.parent))
        from steamdeck_gui import main  # type: ignore
    else:
        raise


if __name__ == '__main__':
    main()



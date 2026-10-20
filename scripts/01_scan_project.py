from __future__ import annotations

import sys
from pathlib import Path


TOOL_ROOT = Path(__file__).resolve().parents[1]
if str(TOOL_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOL_ROOT))

from rosm.scan_project import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())

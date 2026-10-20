from __future__ import annotations

import sys
import unittest
from dataclasses import dataclass
from pathlib import Path


TOOL_ROOT = Path(__file__).resolve().parents[1]
if str(TOOL_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOL_ROOT))

from rosm.snapshot_pose_index import optional_value  # noqa: E402


@dataclass
class FloatOptional:
    value: float
    has_value: bool


class OptionalValueTests(unittest.TestCase):
    def test_preserves_legacy_scalar(self) -> None:
        self.assertEqual(optional_value(0.75), 0.75)

    def test_unwraps_present_optional_value(self) -> None:
        self.assertEqual(optional_value(FloatOptional(value=0.75, has_value=True)), 0.75)

    def test_maps_missing_optional_value_to_none(self) -> None:
        self.assertIsNone(optional_value(FloatOptional(value=0.0, has_value=False)))


if __name__ == "__main__":
    unittest.main()

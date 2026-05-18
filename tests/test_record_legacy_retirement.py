import pathlib
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]


class RecordLegacyRetirementTests(unittest.TestCase):
    def test_legacy_record_entrypoint_is_retired(self):
        launcher = (ROOT / "record" / "run_local_teleop.sh").read_text(encoding="utf-8")

        self.assertFalse((ROOT / "record" / "legacy").exists())
        self.assertIn('"${ROOT_DIR}/main.py"', launcher)
        self.assertNotIn("record/legacy", launcher)
        self.assertNotIn("teleop_keyboard_mouse.py", launcher)


if __name__ == "__main__":
    unittest.main()

"""Dependency-free structural tests for the published source tree."""

import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
USV = ROOT / "ida_ws/src/usv_navigation"


def assigned_int(path, name):
    """Return a literal integer assigned in the module; do not import ROS."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            if any(isinstance(target, ast.Name) and target.id == name for target in node.targets):
                if isinstance(node.value, ast.Constant) and isinstance(node.value.value, (int, float)):
                    return int(node.value.value)
    return None


class PublicationChecks(unittest.TestCase):
    def test_all_python_files_parse(self):
        scripts = list(ROOT.rglob("*.py"))
        self.assertGreaterEqual(len(scripts), 9)
        for script in scripts:
            with self.subTest(path=str(script.relative_to(ROOT))):
                ast.parse(script.read_text(encoding="utf-8"), filename=str(script))

    def test_entrypoints_have_modules(self):
        setup = (USV / "setup.py").read_text(encoding="utf-8")
        for name in ("los_node", "obstacle_avoidance_node", "pid_node",
                     "data_logger_node", "estop", "haberlesme"):
            self.assertIn(f"usv_navigation.{name}:main", setup)
            self.assertTrue((USV / "usv_navigation" / f"{name}.py").is_file())

    def test_protocol_identifiers_align(self):
        receiver = USV / "usv_navigation/haberlesme.py"
        yki = ROOT / "yki/yki_iha_ida_koprusu.py"
        iha = ROOT / "iha/src/canli_plaka_tespit.py"
        self.assertEqual(assigned_int(receiver, "YKI_SISTEM_ID"), 201)
        self.assertEqual(assigned_int(yki, "YKI_SISTEM_ID"), 201)
        self.assertEqual(assigned_int(yki, "IHA_JETSON_SISTEM_ID"), 200)
        self.assertEqual(assigned_int(iha, "MAVLINK_KAYNAK_SISTEM"), 200)
        self.assertEqual(assigned_int(yki, "IDA_JETSON_SISTEM_ID"), 202)
        self.assertEqual(assigned_int(receiver, "IDA_JETSON_SISTEM_ID"), 202)
        self.assertEqual(assigned_int(receiver, "PROTOKOL_SURUMU"), 1)
        self.assertEqual(assigned_int(receiver, "PROTOKOL_IMZASI"), 2026)
        self.assertEqual(assigned_int(yki, "RENK_DOGRULAMA_ADEDI"), 3)
        self.assertIn('"TGT_COLOR"', iha.read_text(encoding="utf-8"))
        self.assertIn('"TGT_COLOR"', yki.read_text(encoding="utf-8"))
        self.assertIn('"/carpilacakhedef"', receiver.read_text(encoding="utf-8"))

    def test_publication_excludes_vendor_repos_and_backups(self):
        self.assertFalse(list(ROOT.rglob(".git")))
        # Python execution legitimately creates caches; .gitignore excludes them.
        self.assertIn("__pycache__/", (ROOT / ".gitignore").read_text())
        self.assertFalse(list(ROOT.rglob("*BACKUP*")))
        self.assertFalse((ROOT / "ida_ws/src/sllidar_ros2").exists())
        self.assertFalse((ROOT / "ida_ws/src/zed-ros2-wrapper").exists())

    def test_no_device_identifiers_from_upload(self):
        scripts = "\n".join(path.read_text(encoding="utf-8") for folder in (ROOT / "iha", ROOT / "yki", USV) for path in folder.rglob("*.py"))
        self.assertIn('os.environ.get("MAVERA_IHA_RFD_SERIAL"', scripts)
        self.assertIn('os.environ.get("MAVERA_IDA_RFD_SERIAL"', scripts)
        self.assertNotIn("serial:///dev/serial/by-id/", (USV / "launch/usv_otonom.launch.py").read_text(encoding="utf-8"))

    def test_model_and_uav_placeholder(self):
        self.assertGreater((USV / "best.pt").stat().st_size, 1_000_000)
        self.assertFalse((ROOT / "iha/models/best.pt").exists())


if __name__ == "__main__":
    unittest.main()

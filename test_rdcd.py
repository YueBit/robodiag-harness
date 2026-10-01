#!/usr/bin/env python3
"""Offline tests for RDCD — Robot Diagnostic Capability Description.

These cover the RDCD v0.1 behavior without ROS 2, rich, or any network access:

- RDCD document loading and validation
- capability applicability (NOT_SUPPORTED vs APPLICABLE)
- runtime availability (AVAILABLE / MISSING / TYPE_MISMATCH)
- system_health aggregation ignoring unsupported capabilities
- SafetyGate respecting RDCD applicability while staying fail-closed

Usage:
    python3 -m unittest test_rdcd        # or:  python3 test_rdcd.py
    pytest test_rdcd.py                  # also works
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import robodiag_core as core
import robodiag_rdcd as rdcd

MINI_PUPPER_YAML = """
schema_version: 0.1

robot:
  id: mini_pupper_2
  name: Mini Pupper 2

capabilities:
  diagnostics:
    topic: /diagnostics
    message_type: diagnostic_msgs/msg/DiagnosticArray
  joint_states:
    topic: /joint_states
    message_type: sensor_msgs/msg/JointState
  imu:
    topic: /imu/data
    message_type: sensor_msgs/msg/Imu
  odometry:
    topic: /odom
    message_type: nav_msgs/msg/Odometry
  velocity_command:
    topic: /cmd_vel
    message_type: geometry_msgs/msg/Twist
"""


def make_rdcd() -> rdcd.Rdcd:
    return rdcd.Rdcd.from_dict(rdcd.parse_rdcd_yaml(MINI_PUPPER_YAML))


class FakeNode:
    """A healthy robot with diagnostics and joint_states but no battery."""

    def inspect_graph(self):
        return {
            "summary": {"nodes": 5, "topics": 10, "services": 4},
            "topics": [
                {"name": "/joint_states", "types": ["sensor_msgs/msg/JointState"]},
                {"name": "/diagnostics", "types": ["diagnostic_msgs/msg/DiagnosticArray"]},
            ],
        }

    def get_diagnostics(self, include_ok=False):
        return {"available": True, "last_array_age_s": 0.1, "statuses": []}

    def battery_state(self):
        return {"available": False}

    def joint_state_cached(self):
        return {"available": True, "age_s": 0.1, "message": {}}

    def sample_topic(self, topic, duration_s=2.5, rate_hz=10.0):
        return {
            "ok": True,
            "samples": 10,
            "effective_rate_hz": 5.0,
            "last": {"_joints": {"lf1": {"position": 0.1}}},
        }

    def inspect_ros2_control(self):
        return {"available": False, "error": "not available"}


class TestRdcdLoading(unittest.TestCase):
    def test_mini_pupper_loads(self):
        r = make_rdcd()
        self.assertEqual(r.robot_id, "mini_pupper_2")
        self.assertEqual(r.name, "Mini Pupper 2")
        self.assertIn("joint_states", r.capabilities)
        self.assertNotIn("battery", r.capabilities)

    def test_load_shipped_file(self):
        path = Path(__file__).resolve().parent / "rdcd" / "mini_pupper_2.yaml"
        r = rdcd.load_rdcd(path)
        self.assertEqual(r.robot_id, "mini_pupper_2")
        self.assertFalse(r.has("battery"))

    def test_unsupported_schema_fails(self):
        data = {"schema_version": "9.9", "robot": {"id": "x"}, "capabilities": {}}
        with self.assertRaises(rdcd.RdcdError):
            rdcd.Rdcd.from_dict(data)

    def test_malformed_capability_fails(self):
        data = {
            "schema_version": "0.1",
            "robot": {"id": "x"},
            "capabilities": {"joint_states": {"topic": "/joint_states"}},
        }
        with self.assertRaises(rdcd.RdcdError):
            rdcd.Rdcd.from_dict(data)


class TestApplicability(unittest.TestCase):
    def test_battery_not_declared_is_not_supported(self):
        r = make_rdcd()
        self.assertFalse(r.has("battery"))
        self.assertEqual(rdcd.capability_status(r, "battery", {}), rdcd.NOT_SUPPORTED)

    def test_joint_states_declared_is_applicable(self):
        r = make_rdcd()
        self.assertTrue(r.has("joint_states"))
        self.assertNotEqual(
            rdcd.capability_status(r, "joint_states", {}), rdcd.NOT_SUPPORTED
        )


class TestRuntimeAvailability(unittest.TestCase):
    def test_available(self):
        r = make_rdcd()
        status = rdcd.capability_status(r, "joint_states", {"/joint_states": ["sensor_msgs/msg/JointState"]})
        self.assertEqual(status, rdcd.AVAILABLE)

    def test_missing(self):
        r = make_rdcd()
        self.assertEqual(rdcd.capability_status(r, "joint_states", {}), rdcd.MISSING)

    def test_type_mismatch(self):
        r = make_rdcd()
        status = rdcd.capability_status(r, "joint_states", {"/joint_states": ["std_msgs/msg/String"]})
        self.assertEqual(status, rdcd.TYPE_MISMATCH)

    def test_not_supported_is_distinct_from_missing(self):
        r = make_rdcd()
        self.assertEqual(rdcd.capability_status(r, "battery", {}), rdcd.NOT_SUPPORTED)
        self.assertEqual(rdcd.capability_status(r, "joint_states", {}), rdcd.MISSING)


class TestTestRunnerWithRdcd(unittest.TestCase):
    def _run(self, test_id, rdcd=None, node=None):
        node = node or FakeNode()
        with tempfile.TemporaryDirectory() as d:
            runner = core.TestRunner(node, core.HistoryStore(Path(d) / "h.db"), rdcd=rdcd)
            return runner.run(test_id)

    def test_battery_health_not_supported(self):
        res = self._run("battery_health", rdcd=make_rdcd())
        self.assertEqual(res["result"], "NOT_SUPPORTED")
        self.assertIs(res["ok"], True)

    def test_system_health_ignores_unsupported(self):
        res = self._run("system_health", rdcd=make_rdcd())
        # graph PASS, diagnostics PASS, joint_states PASS; battery and
        # ros2_control are NOT_SUPPORTED and must not reduce the result.
        self.assertEqual(res["result"], "PASS")
        self.assertIn("battery_health=N/A", res["summary"])
        self.assertIn("ros2_control_health=N/A", res["summary"])

    def test_declared_but_missing_is_a_finding(self):
        node = FakeNode()
        node.sample_topic = lambda *a, **k: {"ok": False, "error": "no data"}
        res = self._run("joint_states_health", rdcd=make_rdcd(), node=node)
        # joint_states is declared, so a missing stream is a FAIL, not N/A.
        self.assertEqual(res["result"], "FAIL")

    def test_generic_mode_still_runs_battery(self):
        res = self._run("battery_health", rdcd=None)
        # No RDCD → battery_health runs and reports SKIP (no BatteryState data).
        self.assertEqual(res["result"], "SKIP")


class TestSafetyGateWithRdcd(unittest.TestCase):
    def _gate(self, node, rdcd):
        return core.SafetyGate(
            node,
            min_battery_pct=0.10,
            min_battery_v=0.0,
            max_diag_age_s=5.0,
            max_joint_age_s=1.0,
            max_battery_age_s=5.0,
            rdcd=rdcd,
        )

    def test_mini_pupper_not_rejected_for_missing_battery(self):
        result = self._gate(FakeNode(), make_rdcd()).check_for_motion()
        self.assertTrue(result["allow"])
        # Battery is not declared, so no battery evidence is required.
        self.assertFalse(any(c["check"].startswith("battery") for c in result["checks"]))

    def test_generic_still_fails_closed_on_missing_battery(self):
        result = self._gate(FakeNode(), None).check_for_motion()
        self.assertFalse(result["allow"])

    def test_declared_but_missing_joint_states_fails_closed(self):
        node = FakeNode()
        node.joint_state_cached = lambda: {"available": False}
        result = self._gate(node, make_rdcd()).check_for_motion()
        self.assertFalse(result["allow"])


if __name__ == "__main__":
    unittest.main(verbosity=2)

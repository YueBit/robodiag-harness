#!/usr/bin/env python3
"""Offline unit tests for the RoboDiag deterministic core (no ROS, no network).

These cover the pieces that must be verifiable in plain CI:

- pure helpers (compact, _jsonable, flatten_numeric, stats_from_samples, _result_ok)
- HistoryStore round-trip
- TestRunner result/ok semantics, diagnostics summary, and system_health aggregation
- SafetyGate fail-closed behaviour and configurable thresholds
- the routing whitelists (JEV_ACTIONS, TEST_CATALOG, build_action_capabilities)

Only ``robodiag_core`` and ``robodiag_jev`` are imported — neither touches
rclpy, rich, or any AI client.

Usage:
    python3 -m unittest test_core        # or:  python3 test_core.py
    pytest test_core.py                  # also works
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import robodiag_core as core
import robodiag_jev as jev
from robodiag_jev import DiagnosticState, JevError, JevRouter, TestResult


class FakeClient:
    """Scripted JevClient stand-in; returns a canned choice for one question."""

    def __init__(self, choice: str) -> None:
        self.choice = choice
        self.model = "fake"

    def system_one(self, state, questions):
        key = next(iter(questions))
        answers = {
            key: {
                "type": "choice",
                "choice": self.choice,
                "confidence": 0.5,
                "probabilities": {self.choice: 1.0},
            }
        }
        return {"answers": answers, "model": self.model}


class FakeHealthyNode:
    def get_diagnostics(self, include_ok=False):
        return {"available": True, "last_array_age_s": 0.1, "statuses": []}

    def battery_state(self):
        return {"available": True, "age_s": 1.0, "percentage": 0.8, "voltage": 15.6}

    def joint_state_cached(self):
        return {"available": True, "age_s": 0.1, "message": {}}

    def inspect_graph(self):
        return {
            "summary": {"nodes": 5, "topics": 10, "services": 4},
            "nodes": [{"name": "n", "namespace": "/"} for _ in range(5)],
            "topics": [
                {"name": "/joint_states", "types": ["sensor_msgs/msg/JointState"]},
                {"name": "/diagnostics", "types": ["diagnostic_msgs/msg/DiagnosticArray"]},
            ],
            "services": [],
        }

    def sample_topic(self, topic, duration_s=5.0, rate_hz=5.0):
        return {
            "ok": True,
            "samples": 10,
            "effective_rate_hz": 5.0,
            "last": {"_joints": {"lf1": {"position": 0.1}}},
        }

    def inspect_ros2_control(self):
        return {"available": False, "error": "not available"}


class TestPureHelpers(unittest.TestCase):
    def test_compact_preserves_failed_status_on_truncation(self):
        value = {"ok": False, "error": "boom", "data": "x" * 20000}
        out = json.loads(core.compact(value))
        self.assertTrue(out["truncated"])
        self.assertIs(out["ok"], False)
        self.assertEqual(out["error"], "boom")

    def test_compact_preserves_ok_and_identity_on_truncation(self):
        value = {"ok": True, "test_id": "graph_health", "result": "PASS", "data": "y" * 20000}
        out = json.loads(core.compact(value))
        self.assertTrue(out["truncated"])
        self.assertIs(out["ok"], True)
        self.assertEqual(out["test_id"], "graph_health")
        self.assertEqual(out["result"], "PASS")

    def test_compact_short_value_passthrough(self):
        value = {"ok": True, "n": 1}
        self.assertEqual(core.compact(value), json.dumps(value))

    def test_jsonable_sanitises_nonfinite_floats(self):
        out = core._jsonable({"nan": float("nan"), "inf": float("inf"), "x": 1.0})
        self.assertIsNone(out["nan"])
        self.assertIsNone(out["inf"])
        self.assertEqual(out["x"], 1.0)

    def test_flatten_and_stats(self):
        samples = [
            {"a": 1, "b": {"c": 2}},
            {"a": 3, "b": {"c": 4}},
        ]
        stats = core.stats_from_samples(samples)
        self.assertEqual(stats["a"]["min"], 1.0)
        self.assertEqual(stats["a"]["max"], 3.0)
        self.assertEqual(stats["a"]["mean"], 2.0)
        self.assertEqual(stats["b.c"]["range"], 2.0)

    def test_result_ok_semantics(self):
        # A completed FAIL test is meaningful evidence.
        self.assertTrue(
            core._result_ok(json.dumps({"ok": True, "test_id": "diagnostics_health", "result": "FAIL"}))
        )
        # A genuine call failure is not.
        self.assertFalse(core._result_ok(json.dumps({"ok": False, "error": "boom"})))
        self.assertFalse(core._result_ok(json.dumps({"ok": True, "error": "boom"})))
        self.assertFalse(core._result_ok("[1, 2, 3]"))
        self.assertFalse(core._result_ok("not json"))


class TestHistoryStore(unittest.TestCase):
    def test_roundtrip(self):
        with tempfile.TemporaryDirectory() as d:
            store = core.HistoryStore(Path(d) / "h.db")
            tr = TestResult(
                test_id="graph_health",
                result="PASS",
                summary="ok",
                evidence=[],
                started_at="2026-01-01T00:00:00+0000",
                duration_ms=1,
            )
            store.add(tr)
            rows = store.recent(10)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["test_id"], "graph_health")
            self.assertEqual(rows[0]["result"], "PASS")


class TestTestRunner(unittest.TestCase):
    def test_fail_counts_as_meaningful_ok(self):
        node = FakeHealthyNode()
        node.get_diagnostics = lambda include_ok=True: {
            "available": True,
            "last_array_age_s": 0.1,
            "statuses": [{"name": "ekf: odometry/filtered", "level": 2, "message": "No events recorded."}],
        }
        with tempfile.TemporaryDirectory() as d:
            runner = core.TestRunner(node, core.HistoryStore(Path(d) / "h.db"))
            res = runner.run("diagnostics_health")
        self.assertIs(res["ok"], True)
        self.assertEqual(res["result"], "FAIL")
        # Fix #7: the summary folds in the failing component name and message.
        self.assertIn("ekf", res["summary"])
        self.assertIn("No events recorded.", res["summary"])

    def test_unknown_test_id_is_a_call_failure(self):
        with tempfile.TemporaryDirectory() as d:
            runner = core.TestRunner(FakeHealthyNode(), core.HistoryStore(Path(d) / "h.db"))
            res = runner.run("does_not_exist")
        self.assertIs(res["ok"], False)
        self.assertIn("error", res)

    def test_severity_ranking(self):
        self.assertLess(core.TestRunner._severity("PASS"), core.TestRunner._severity("SKIP"))
        self.assertLess(core.TestRunner._severity("SKIP"), core.TestRunner._severity("WARN"))
        self.assertLess(core.TestRunner._severity("WARN"), core.TestRunner._severity("FAIL"))

    def test_system_health_aggregation(self):
        with tempfile.TemporaryDirectory() as d:
            runner = core.TestRunner(FakeHealthyNode(), core.HistoryStore(Path(d) / "h.db"))
            res = runner.run("system_health")
        # FakeHealthyNode: graph PASS, diagnostics SKIP (no data), joints PASS,
        # ros2_control SKIP → worst is SKIP (severity 1) → overall PASS.
        self.assertEqual(res["result"], "PASS")
        self.assertIs(res["ok"], True)
        self.assertIn("graph_health=PASS", res["summary"])


class TestSafetyGate(unittest.TestCase):
    def _gate(self, node, **kwargs):
        return core.SafetyGate(
            node,
            min_battery_pct=0.10,
            min_battery_v=0.0,
            max_diag_age_s=5.0,
            max_joint_age_s=1.0,
            **kwargs,
        )

    def test_fail_closed_when_evidence_missing(self):
        class Empty:
            def get_diagnostics(self, include_ok=False):
                return {"available": False}

            def battery_state(self):
                return {"available": False}

            def joint_state_cached(self):
                return {"available": False}

        self.assertFalse(self._gate(Empty()).check_for_motion()["allow"])

    def test_healthy_allows(self):
        self.assertTrue(self._gate(FakeHealthyNode()).check_for_motion()["allow"])

    def test_critical_diagnostics_deny(self):
        node = FakeHealthyNode()
        node.get_diagnostics = lambda include_ok=False: {
            "available": True,
            "last_array_age_s": 0.1,
            "statuses": [{"name": "ekf", "level": core.DIAG_ERROR, "message": "No events recorded."}],
        }
        self.assertFalse(self._gate(node).check_for_motion()["allow"])

    def test_stale_battery_denies(self):
        node = FakeHealthyNode()
        node.battery_state = lambda: {"available": True, "age_s": 6.0, "percentage": 0.8, "voltage": 15.6}
        self.assertFalse(self._gate(node).check_for_motion()["allow"])

    def test_battery_age_threshold_is_configurable(self):
        node = FakeHealthyNode()
        node.battery_state = lambda: {"available": True, "age_s": 3.0, "percentage": 0.8, "voltage": 15.6}
        self.assertTrue(self._gate(node, max_battery_age_s=10.0).check_for_motion()["allow"])
        self.assertFalse(self._gate(node, max_battery_age_s=2.0).check_for_motion()["allow"])


class TestWhitelists(unittest.TestCase):
    def test_jev_actions_exclude_emergency_stop(self):
        self.assertNotIn("emergency_stop", jev.JEV_ACTIONS)

    def test_test_catalog_is_read_only(self):
        self.assertTrue(core.TEST_CATALOG)
        for spec in core.TEST_CATALOG.values():
            self.assertFalse(spec.get("writes"))

    def test_action_capabilities_only_expose_read_only_tests(self):
        caps = core.build_action_capabilities()
        self.assertIn("emergency_stop", caps)
        for test_id in core.TEST_CATALOG:
            self.assertIn(f"run_test:{test_id}", caps)

        # A write test must never appear in the ACTION registry.
        with mock.patch.dict(core.TEST_CATALOG, {"motion_test": {"name": "motion", "description": "x", "writes": True}}):
            caps2 = core.build_action_capabilities()
        self.assertNotIn("run_test:motion_test", caps2)

    def test_next_action_exclude_removes_actions(self):
        router = JevRouter(FakeClient("get_diagnostics"))
        # Excluding everything except finalize (which is gated) must raise.
        excluded = set(jev.JEV_ACTIONS) - {"finalize"}
        with self.assertRaises(JevError):
            router.next_action(DiagnosticState(symptom="x"), allow_finalize=False, exclude=excluded)

        # A normal exclusion is accepted and the scripted choice still returns.
        d = router.next_action(DiagnosticState(symptom="x"), exclude={"sample_topic"})
        self.assertEqual(d.action, "get_diagnostics")


if __name__ == "__main__":
    unittest.main(verbosity=2)

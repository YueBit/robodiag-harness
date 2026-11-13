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
from robodiag_jev import (
    DiagnosticState,
    JevError,
    JevRouter,
    TestResult,
    ToolCall,
    parse_verification_plan,
    unmet_evidence_requirements,
    validated_verification_checks,
)


class FakeClient:
    """Scripted JevClient stand-in; returns a canned choice for one question."""

    def __init__(self, choice: str) -> None:
        self.choice = choice
        self.model = "fake"
        self.last_questions = None

    def system_one(self, state, questions):
        self.last_questions = questions
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

    def test_is_constant_zero(self):
        self.assertTrue(core.is_constant_zero({"min": 0.0, "max": 0, "mean": 0.0, "std": 0.0, "range": 0.0}))
        self.assertFalse(core.is_constant_zero({"min": 0.0, "max": 1.0, "mean": 0.5, "std": 0.1, "range": 1.0}))
        self.assertFalse(core.is_constant_zero({"min": 5.0, "max": 5.0, "mean": 5.0, "std": 0.0, "range": 0.0}))
        self.assertFalse(core.is_constant_zero({"min": "n/a"}))

    def test_is_verify_request(self):
        self.assertTrue(core.is_verify_request("/verify"))
        self.assertTrue(core.is_verify_request("continue verify"))
        self.assertTrue(core.is_verify_request("Continue to verify, please"))
        self.assertTrue(core.is_verify_request("verify more"))
        self.assertTrue(core.is_verify_request("yes, keep verifying"))
        self.assertFalse(core.is_verify_request("verify the battery"))
        self.assertFalse(core.is_verify_request("anything wrong with this robot?"))
        self.assertFalse(core.is_verify_request("run system_health"))

    def test_sample_activity_summary(self):
        result = {
            "ok": True,
            "topic": "/odom",
            "samples": 24,
            "effective_rate_hz": 4.8,
            "statistics": {
                "pose.covariance[0]": {"min": 1.0, "max": 9.0, "mean": 5.0, "std": 2.0, "range": 8.0},
                "pose.covariance[1]": {"min": 0.0, "max": 0.0, "mean": 0.0, "std": 0.0, "range": 0.0},
            },
        }
        summary = core.sample_activity_summary(result)
        self.assertIn("/odom", summary)
        self.assertIn("24 samples @ 4.8 Hz", summary)
        self.assertIn("pose.covariance[0]", summary)
        self.assertNotIn("pose.covariance[1]", summary)  # constant zero is not "variable"
        failed = core.sample_activity_summary({"ok": False, "topic": "/odom", "error": "timeout"})
        self.assertIn("timeout", failed)

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
        # FakeHealthyNode: graph PASS, diagnostics PASS, battery PASS, joints
        # PASS, ros2_control SKIP → worst is SKIP (severity 1) → overall PASS.
        self.assertEqual(res["result"], "PASS")
        self.assertIs(res["ok"], True)
        self.assertIn("graph_health=PASS", res["summary"])
        self.assertIn("battery_health=PASS", res["summary"])


class TestBatteryHealth(unittest.TestCase):
    def _run(self, battery: dict) -> dict:
        node = mock.Mock()
        node.battery_state.return_value = battery
        with tempfile.TemporaryDirectory() as d:
            runner = core.TestRunner(node, core.HistoryStore(Path(d) / "h.db"))
            return runner.run("battery_health")

    def test_healthy_passes(self):
        res = self._run(
            {"available": True, "age_s": 0.5, "percentage": 0.8, "voltage": 15.6, "present": True}
        )
        self.assertEqual(res["result"], "PASS")

    def test_missing_skips(self):
        res = self._run({"available": False})
        self.assertEqual(res["result"], "SKIP")

    def test_stale_fails(self):
        res = self._run({"available": True, "age_s": 7.0, "percentage": 0.8, "voltage": 15.6})
        self.assertEqual(res["result"], "FAIL")

    def test_invalid_percentage_fails(self):
        res = self._run({"available": True, "age_s": 0.5, "percentage": None, "voltage": 15.6})
        self.assertEqual(res["result"], "FAIL")

    def test_invalid_voltage_fails(self):
        res = self._run({"available": True, "age_s": 0.5, "percentage": 0.8, "voltage": None})
        self.assertEqual(res["result"], "FAIL")

    def test_low_battery_warns(self):
        res = self._run(
            {"available": True, "age_s": 0.5, "percentage": 0.05, "voltage": 15.6, "present": True}
        )
        self.assertEqual(res["result"], "WARN")

    def test_not_present_warns(self):
        res = self._run(
            {"available": True, "age_s": 0.5, "percentage": 0.8, "voltage": 15.6, "present": False}
        )
        self.assertEqual(res["result"], "WARN")


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

    def test_required_evidence_helper(self):
        self.assertEqual(len(unmet_evidence_requirements([])), 2)
        only_test = [ToolCall(tool="run_test", args={}, result="{}", ok=True)]
        unmet = unmet_evidence_requirements(only_test)
        self.assertEqual(unmet, [("sample_topic", "get_topic_snapshot")])
        # An attempted-but-failed measurement still counts as attempted so the
        # loop cannot deadlock on an unavailable topic.
        failed_sample = [
            ToolCall(tool="run_test", args={}, result="{}", ok=True),
            ToolCall(tool="sample_topic", args={}, result='{"ok": false}', ok=False),
        ]
        self.assertEqual(unmet_evidence_requirements(failed_sample), [])
        snapshot = [ToolCall(tool="get_topic_snapshot", args={}, result="{}", ok=True)]
        self.assertEqual(unmet_evidence_requirements(snapshot), [("run_test",)])

    def test_parse_verification_plan(self):
        self.assertEqual(parse_verification_plan('{"checks": [{"tool": "inspect_graph"}]}'), [{"tool": "inspect_graph"}])
        self.assertEqual(parse_verification_plan('```json\n{"checks": []}\n```'), [])
        self.assertEqual(parse_verification_plan("not json"), [])
        self.assertEqual(parse_verification_plan('{"checks": "nope"}'), [])

    def test_validated_verification_checks_fails_closed(self):
        catalog = {
            "graph_health": {"writes": False},
            "motion_test": {"writes": True},
        }
        proposals = [
            {"tool": "emergency_stop"},
            {"tool": "finalize"},
            {"tool": "does_not_exist"},
            {"tool": "run_test", "args": {"test_id": "motion_test"}},
            {"tool": "run_test", "args": {"test_id": "unknown"}},
            {"tool": "sample_topic", "args": {"topic": "/not/in/graph"}},
            {"tool": "sample_topic", "args": {"topic": "relative"}},
            {"tool": "sample_topic", "args": {"topic": "/imu/data"}},
            {"tool": "run_test", "args": {"test_id": "graph_health"}},
        ]
        checks = validated_verification_checks(
            proposals, [], catalog, {"/imu/data"}, max_checks=10
        )
        self.assertEqual(
            checks,
            [("sample_topic", {"topic": "/imu/data", "duration_s": 3.0, "rate_hz": 5.0}),
             ("run_test", {"test_id": "graph_health"})],
        )

    def test_validated_verification_checks_dedups_and_caps(self):
        catalog = {"graph_health": {"writes": False}}
        already = [ToolCall(tool="inspect_graph", args={}, result="{}", ok=True)]
        proposals = [
            {"tool": "inspect_graph"},
            {"tool": "get_diagnostics"},
            {"tool": "inspect_ros2_control"},
            {"tool": "query_history"},
        ]
        checks = validated_verification_checks(proposals, already, catalog, set(), max_checks=2)
        self.assertEqual(len(checks), 2)
        self.assertNotIn("inspect_graph", [tool for tool, _ in checks])

    def test_next_action_reports_required_categories(self):
        client = FakeClient("run_test")
        router = JevRouter(client)
        router.next_action(
            DiagnosticState(symptom="x"), allow_finalize=False, required=["run_test"]
        )
        instructions = client.last_questions["next_action"]["instructions"]
        self.assertIn("run_test", instructions)
        self.assertNotIn("finalize", client.last_questions["next_action"]["criteria"])

    def test_next_action_exclude_removes_actions(self):
        router = JevRouter(FakeClient("get_diagnostics"))
        # Excluding everything except finalize (which is gated) must raise.
        excluded = set(jev.JEV_ACTIONS) - {"finalize"}
        with self.assertRaises(JevError):
            router.next_action(DiagnosticState(symptom="x"), allow_finalize=False, exclude=excluded)

        # A normal exclusion is accepted and the scripted choice still returns.
        d = router.next_action(DiagnosticState(symptom="x"), exclude={"sample_topic"})
        self.assertEqual(d.action, "get_diagnostics")


class FakeSensorNode:
    """Returns scripted sample_topic results keyed by absolute topic name."""

    def __init__(self, **by_topic):
        self.by_topic = by_topic

    def sample_topic(self, topic, duration_s=5.0, rate_hz=5.0):
        return self.by_topic.get(topic, {"ok": False, "error": "no samples"})


def _flat_stat(max_val: float) -> dict:
    return {"min": 0.0, "max": max_val, "mean": 0.0, "std": 0.0, "range": max_val}


def _cmd_sample(linear_x_max: float = 0.0) -> dict:
    return {
        "ok": True,
        "samples": 20,
        "effective_rate_hz": 20.0,
        "statistics": {
            "linear.x": _flat_stat(linear_x_max),
            "linear.y": _flat_stat(0.0),
            "linear.z": _flat_stat(0.0),
            "angular.x": _flat_stat(0.0),
            "angular.y": _flat_stat(0.0),
            "angular.z": _flat_stat(0.0),
        },
    }


def _odom_motion_sample(linear_x_max: float = 0.0) -> dict:
    return {
        "ok": True,
        "samples": 20,
        "effective_rate_hz": 20.0,
        "statistics": {
            "twist.twist.linear.x": _flat_stat(linear_x_max),
            "twist.twist.linear.y": _flat_stat(0.0),
            "twist.twist.linear.z": _flat_stat(0.0),
            "twist.twist.angular.x": _flat_stat(0.0),
            "twist.twist.angular.y": _flat_stat(0.0),
            "twist.twist.angular.z": _flat_stat(0.0),
        },
    }


class TestBaselineStore(unittest.TestCase):
    def _store(self) -> core.BaselineStore:
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        return core.BaselineStore(Path(d.name) / "b.db")

    def test_observe_and_compare(self):
        b = self._store()
        for v in (30.0, 31.0, 29.0, 30.0, 31.0):
            b.observe("joint_states.rate_hz", v)
        c = b.compare("joint_states.rate_hz", 30.0)
        self.assertEqual(c["n"], 5)
        self.assertAlmostEqual(c["mean"], 30.2, places=5)
        self.assertLess(c["sigma"], 1.0)  # near the mean
        drifted = b.compare("joint_states.rate_hz", 45.0)
        self.assertGreater(drifted["sigma"], 10.0)

    def test_zero_variance_deviation_is_huge(self):
        b = self._store()
        for _ in range(5):
            b.observe("m", 30.0)
        self.assertEqual(b.compare("m", 30.0)["sigma"], 0.0)
        self.assertGreater(b.compare("m", 31.0)["sigma"], 1e6)

    def test_stats_and_clear(self):
        b = self._store()
        b.observe("a", 1.0)
        b.observe("b", 2.0)
        self.assertEqual(len(b.stats()), 2)
        b.clear()
        self.assertEqual(b.stats(), [])

    def test_rejects_nonfinite(self):
        b = self._store()
        with self.assertRaises(ValueError):
            b.observe("m", float("nan"))


class TestBaselineHealth(unittest.TestCase):
    @staticmethod
    def _joint_sample(rate: float) -> dict:
        return {"ok": True, "samples": 10, "effective_rate_hz": rate,
                "last": {"_joints": {"a": {"position": 0.0}}}}

    def _runner(self, node, baseline) -> core.TestRunner:
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        return core.TestRunner(
            node, core.HistoryStore(Path(d.name) / "h.db"), baseline=baseline
        )

    def test_not_configured_skips(self):
        node = FakeSensorNode(**{"/joint_states": self._joint_sample(30.0)})
        with tempfile.TemporaryDirectory() as d:
            runner = core.TestRunner(node, core.HistoryStore(Path(d) / "h.db"), baseline=None)
            res = runner.run("baseline_health")
        self.assertEqual(res["result"], "SKIP")

    def test_not_established_skips(self):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        baseline = core.BaselineStore(Path(d.name) / "b.db")
        node = FakeSensorNode(**{"/joint_states": self._joint_sample(30.0)})
        res = self._runner(node, baseline).run("baseline_health")
        self.assertEqual(res["result"], "SKIP")
        self.assertIn("Baseline not established", res["summary"])
        self.assertIn("5 more time(s)", res["summary"])

    def test_warmup_reports_remaining_runs(self):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        baseline = core.BaselineStore(Path(d.name) / "b.db")
        node = FakeSensorNode(**{"/joint_states": self._joint_sample(30.0)})
        runner = self._runner(node, baseline)
        for remaining in (5, 4, 3, 2, 1):
            res = runner.run("baseline_health")
            self.assertEqual(res["result"], "SKIP")
            self.assertIn(f"{remaining} more time(s)", res["summary"])
        res = runner.run("baseline_health")
        self.assertIn(res["result"], ("PASS", "WARN", "FAIL"), res["summary"])

    def test_stable_passes(self):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        baseline = core.BaselineStore(Path(d.name) / "b.db")
        for _ in range(6):
            baseline.observe("joint_states.rate_hz", 30.0)
        node = FakeSensorNode(**{"/joint_states": self._joint_sample(30.0)})
        res = self._runner(node, baseline).run("baseline_health")
        self.assertEqual(res["result"], "PASS")

    def test_drift_fails(self):
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        baseline = core.BaselineStore(Path(d.name) / "b.db")
        for _ in range(6):
            baseline.observe("joint_states.rate_hz", 30.0)
        node = FakeSensorNode(**{"/joint_states": self._joint_sample(90.0)})
        res = self._runner(node, baseline).run("baseline_health")
        self.assertEqual(res["result"], "FAIL")
        self.assertIn("joint_states.rate_hz", res["summary"])


class TestOdometryVsImu(unittest.TestCase):
    @staticmethod
    def _odom(roll_deg: float = 0.0, ang_z: float = 0.0) -> dict:
        import math
        r = math.radians(roll_deg) / 2.0
        qx, qw = math.sin(r), math.cos(r)
        return {"ok": True, "samples": 10, "last": {
            "pose": {"pose": {"orientation": {"x": qx, "y": 0.0, "z": 0.0, "w": qw}}},
            "twist": {"twist": {"angular": {"z": ang_z}}},
        }}

    @staticmethod
    def _imu(roll_deg: float = 0.0, gyro_z: float = 0.0) -> dict:
        import math
        r = math.radians(roll_deg) / 2.0
        qx, qw = math.sin(r), math.cos(r)
        return {"ok": True, "samples": 10, "last": {
            "orientation": {"x": qx, "y": 0.0, "z": 0.0, "w": qw},
            "angular_velocity": {"z": gyro_z},
        }}

    def _runner(self, node) -> core.TestRunner:
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        return core.TestRunner(node, core.HistoryStore(Path(d.name) / "h.db"))

    def test_agreement_passes(self):
        node = FakeSensorNode(**{"/odom": self._odom(), "/imu/data": self._imu()})
        self.assertEqual(self._runner(node).run("odometry_vs_imu")["result"], "PASS")

    def test_roll_mismatch_warns(self):
        node = FakeSensorNode(**{"/odom": self._odom(0.0), "/imu/data": self._imu(10.0)})
        self.assertEqual(self._runner(node).run("odometry_vs_imu")["result"], "WARN")

    def test_roll_mismatch_fails(self):
        node = FakeSensorNode(**{"/odom": self._odom(0.0), "/imu/data": self._imu(20.0)})
        self.assertEqual(self._runner(node).run("odometry_vs_imu")["result"], "FAIL")

    def test_missing_stream_skips_in_generic_mode(self):
        node = FakeSensorNode(**{"/odom": {"ok": False, "error": "no samples"}})
        self.assertEqual(self._runner(node).run("odometry_vs_imu")["result"], "SKIP")

    def test_missing_stream_fails_when_declared(self):
        import robodiag_rdcd as rdcd
        caps = {
            "odometry": rdcd.Capability("odometry", "/odom", "nav_msgs/msg/Odometry"),
            "imu": rdcd.Capability("imu", "/imu/data", "sensor_msgs/msg/Imu"),
        }
        rdcd_obj = rdcd.Rdcd("r", "R", caps)
        node = FakeSensorNode(**{"/odom": {"ok": False, "error": "no samples"}})
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        runner = core.TestRunner(
            node, core.HistoryStore(Path(d.name) / "h.db"), rdcd=rdcd_obj
        )
        self.assertEqual(runner.run("odometry_vs_imu")["result"], "FAIL")


class TestCmdVelVsMotion(unittest.TestCase):
    def _runner(self, node) -> core.TestRunner:
        d = tempfile.TemporaryDirectory()
        self.addCleanup(d.cleanup)
        return core.TestRunner(node, core.HistoryStore(Path(d.name) / "h.db"))

    def _run(self, cmd_peak: float, odom_peak: float) -> dict:
        node = FakeSensorNode(**{
            "/cmd_vel": _cmd_sample(cmd_peak),
            "/odom": _odom_motion_sample(odom_peak),
        })
        return self._runner(node).run("cmd_vel_vs_motion")

    def test_at_rest_passes(self):
        self.assertEqual(self._run(0.0, 0.0)["result"], "PASS")

    def test_command_without_motion_fails(self):
        self.assertEqual(self._run(0.5, 0.0)["result"], "FAIL")

    def test_under_response_warns(self):
        self.assertEqual(self._run(0.5, 0.05)["result"], "WARN")

    def test_matched_response_passes(self):
        self.assertEqual(self._run(0.5, 0.45)["result"], "PASS")


class TestPluginTests(unittest.TestCase):
    def test_register_and_run(self):
        def my_test(runner):
            return "PASS", "plugin ran", [{"source": "plugin", "metric": "x", "value": 1}]

        core.register_test("my_plugin_test", "My plugin", "desc", my_test)
        self.addCleanup(core.TEST_CATALOG.pop, "my_plugin_test", None)
        self.addCleanup(core._PLUGIN_TEST_FNS.pop, "my_plugin_test", None)
        with tempfile.TemporaryDirectory() as d:
            runner = core.TestRunner(FakeSensorNode(), core.HistoryStore(Path(d) / "h.db"))
            res = runner.run("my_plugin_test")
        self.assertEqual(res["result"], "PASS")
        self.assertEqual(res["summary"], "plugin ran")

    def test_plugin_multi_require_not_supported(self):
        import robodiag_rdcd as rdcd

        def my_test(runner):
            return "PASS", "ok", []

        core.register_test("multi_req_test", "Multi", "desc", my_test, requires=["odometry", "imu"])
        self.addCleanup(core.TEST_CATALOG.pop, "multi_req_test", None)
        self.addCleanup(core._PLUGIN_TEST_FNS.pop, "multi_req_test", None)
        partial = rdcd.Rdcd(
            "r", "R",
            {"odometry": rdcd.Capability("odometry", "/odom", "nav_msgs/msg/Odometry")},
        )
        with tempfile.TemporaryDirectory() as d:
            runner = core.TestRunner(
                FakeSensorNode(), core.HistoryStore(Path(d) / "h.db"), rdcd=partial
            )
            res = runner.run("multi_req_test")
        self.assertEqual(res["result"], "NOT_SUPPORTED")
        self.assertIn("imu", res["summary"])

    def test_load_entry_point_tests(self):
        class _EP:
            name = "fake_test"

            def load(self):
                def fn(runner):
                    return "PASS", "from entry point", []
                fn.name = "Fake"
                fn.description = "d"
                return fn

        with mock.patch("importlib.metadata.entry_points", return_value=[_EP()]):
            loaded = core.load_entry_point_tests()
        self.assertEqual(loaded, ["fake_test"])
        self.assertIn("fake_test", core.TEST_CATALOG)
        self.addCleanup(core.TEST_CATALOG.pop, "fake_test", None)
        self.addCleanup(core._PLUGIN_TEST_FNS.pop, "fake_test", None)


if __name__ == "__main__":
    unittest.main(verbosity=2)

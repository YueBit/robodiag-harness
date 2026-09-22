#!/usr/bin/env python3
"""RoboDiag deterministic core — no ROS, no rich, no network.

This module holds the pieces of the harness that must stay testable without a
ROS 2 environment: JSON/statistics helpers, the SQLite history store, the
deterministic test catalog and runner, the motion Safety Gate, and the ACTION
capability registry. It imports only the stdlib and ``robodiag_jev`` (which is
also stdlib-only), so it can be unit-tested in plain CI.

    Deterministic tools decide what is true.
    The Safety Gate decides what is allowed.

Nothing in this module imports rclpy, rich, or any AI client.
"""

from __future__ import annotations

import json
import math
import sqlite3
import threading
import time
from array import array
from dataclasses import asdict
from pathlib import Path
from typing import Any

from robodiag_jev import Evidence, QueryCapability, TestResult

# DiagnosticStatus.level is a `byte` field; on ROS 2 Humble the class constants
# are `bytes` (b'\x00'..b'\x03'), so use plain ints internally.
DIAG_OK = 0
DIAG_WARN = 1
DIAG_ERROR = 2
DIAG_STALE = 3


def now_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S%z")


def _jsonable(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, array)):
        return [_jsonable(v) for v in value]
    if hasattr(value, "tolist"):
        try:
            return _jsonable(value.tolist())
        except Exception:
            pass
    if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
        return None
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def compact(value: Any, limit: int = 12000) -> str:
    try:
        s = json.dumps(value, ensure_ascii=False, allow_nan=False, default=str)
    except (TypeError, ValueError):
        s = json.dumps(_jsonable(value), ensure_ascii=False, default=str)
    if len(s) <= limit:
        return s
    # Truncation must never fabricate a healthy status. Preserve the real
    # top-level status/identity keys so _result_ok() and the LLM explainer still
    # see the true outcome of an oversized call instead of a synthesized ok=true.
    envelope: dict[str, Any] = {"truncated": True, "preview": s[: limit - 200]}
    if isinstance(value, dict):
        for key in (
            "ok",
            "error",
            "topic",
            "test_id",
            "result",
            "available",
            "summary",
            "warning",
        ):
            if key in value:
                envelope[key] = value[key]
        if "ok" not in value:
            envelope["ok"] = True
    else:
        envelope["ok"] = True
    return json.dumps(envelope, ensure_ascii=False, default=str)


def flatten_numeric(value: Any, prefix: str = "", max_array_items: int = 64) -> dict[str, float]:
    """Flatten numeric leaves for statistics; large arrays are intentionally capped."""
    out: dict[str, float] = {}
    if isinstance(value, bool):
        return out
    if isinstance(value, (int, float)):
        if math.isfinite(float(value)):
            out[prefix or "value"] = float(value)
        return out
    if isinstance(value, dict):
        for key, sub in value.items():
            p = f"{prefix}.{key}" if prefix else str(key)
            out.update(flatten_numeric(sub, p, max_array_items))
        return out
    if isinstance(value, (list, tuple)):
        for i, sub in enumerate(value[:max_array_items]):
            p = f"{prefix}[{i}]"
            out.update(flatten_numeric(sub, p, max_array_items))
    return out


def stats_from_samples(samples: list[dict[str, Any]]) -> dict[str, dict[str, float | int]]:
    buckets: dict[str, list[float]] = {}
    for sample in samples:
        for path, value in flatten_numeric(sample).items():
            buckets.setdefault(path, []).append(value)

    result: dict[str, dict[str, float | int]] = {}
    for path, values in buckets.items():
        if not values:
            continue
        mean = sum(values) / len(values)
        variance = sum((v - mean) ** 2 for v in values) / len(values)
        result[path] = {
            "n": len(values),
            "min": round(min(values), 6),
            "max": round(max(values), 6),
            "mean": round(mean, 6),
            "std": round(math.sqrt(variance), 6),
            "range": round(max(values) - min(values), 6),
        }
    return result


def _result_ok(result: str) -> bool:
    """A tool call counts as meaningful evidence only if it produced a result.

    A FAILing diagnostic test still produced a result (with ``result: "FAIL"``),
    so it must be treated as meaningful evidence; only calls that errored out or
    returned ``ok: false`` without a result are excluded.
    """
    try:
        data = json.loads(result)
    except (ValueError, TypeError):
        return False
    if not isinstance(data, dict):
        return False
    # run_test reports ok=True for every completed test (see TestRunner.run);
    # its severity lives in data["result"], not in data["ok"].
    return data.get("ok") is not False and not data.get("error")


# ──────────────────────────────────────────────────────────────────────────────
# SQLite history
# ──────────────────────────────────────────────────────────────────────────────
class HistoryStore:
    def __init__(self, path: Path):
        self.path = path.expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        con = self._connect()
        try:
            with con:
                con.execute(
                    """
                    CREATE TABLE IF NOT EXISTS test_history (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        started_at TEXT NOT NULL,
                        test_id TEXT NOT NULL,
                        result TEXT NOT NULL,
                        summary TEXT NOT NULL,
                        duration_ms INTEGER NOT NULL,
                        payload TEXT NOT NULL
                    )
                    """
                )
        finally:
            con.close()

    def _connect(self) -> sqlite3.Connection:
        return sqlite3.connect(str(self.path), timeout=3.0)

    def add(self, result: TestResult) -> None:
        with self._lock:
            con = self._connect()
            try:
                with con:
                    con.execute(
                        """INSERT INTO test_history
                           (started_at, test_id, result, summary, duration_ms, payload)
                           VALUES (?, ?, ?, ?, ?, ?)""",
                        (
                            result.started_at,
                            result.test_id,
                            result.result,
                            result.summary,
                            result.duration_ms,
                            json.dumps(result.as_dict(), ensure_ascii=False, default=str),
                        ),
                    )
            finally:
                con.close()

    def recent(self, limit: int = 10) -> list[dict[str, Any]]:
        limit = max(1, min(int(limit), 100))
        with self._lock:
            con = self._connect()
            try:
                with con:
                    rows = con.execute(
                        """SELECT id, started_at, test_id, result, summary, duration_ms
                           FROM test_history ORDER BY id DESC LIMIT ?""",
                        (limit,),
                    ).fetchall()
            finally:
                con.close()
        return [
            {
                "id": row[0],
                "started_at": row[1],
                "test_id": row[2],
                "result": row[3],
                "summary": row[4],
                "duration_ms": row[5],
            }
            for row in rows
        ]


# ──────────────────────────────────────────────────────────────────────────────
# Safety Gate
# ──────────────────────────────────────────────────────────────────────────────
class SafetyGate:
    def __init__(
        self,
        node: Any,
        min_battery_pct: float,
        min_battery_v: float,
        max_diag_age_s: float,
        max_joint_age_s: float,
        max_battery_age_s: float = 5.0,
    ):
        self.node = node
        self.min_battery_pct = min_battery_pct
        self.min_battery_v = min_battery_v
        self.max_diag_age_s = max_diag_age_s
        self.max_joint_age_s = max_joint_age_s
        self.max_battery_age_s = max_battery_age_s

    def check_for_motion(self) -> dict[str, Any]:
        """Fail closed: deny motion unless required evidence is present and healthy.

        Missing evidence (no /diagnostics, battery, or /joint_states) is treated
        as unsafe, never as "normal". This matches the design principle that
        "no data" must not permit motion.
        """
        checks: list[dict[str, Any]] = []
        allow = True

        diag = self.node.get_diagnostics(include_ok=False)
        if diag.get("available"):
            age = diag.get("last_array_age_s")
            if age is not None and age > self.max_diag_age_s:
                allow = False
                checks.append({"check": "diagnostics_fresh", "ok": False, "reason": f"/diagnostics stale: {age}s"})
            else:
                bad = [x for x in diag["statuses"] if x["level"] >= DIAG_ERROR]
                if bad:
                    allow = False
                    checks.append({"check": "diagnostics", "ok": False, "reason": f"critical diagnostic statuses: {len(bad)}", "items": bad[:5]})
                else:
                    checks.append({"check": "diagnostics", "ok": True})
        else:
            allow = False
            checks.append({"check": "diagnostics", "ok": False, "reason": "no /diagnostics evidence (missing)"})

        battery = self.node.battery_state()
        if battery.get("available"):
            if battery.get("age_s", 999) > self.max_battery_age_s:
                allow = False
                checks.append({"check": "battery_fresh", "ok": False, "reason": f"battery state stale: {battery['age_s']}s"})
            else:
                pct = battery.get("percentage")
                voltage = battery.get("voltage")
                if pct is not None and self.min_battery_pct > 0 and pct < self.min_battery_pct:
                    allow = False
                    checks.append({"check": "battery_pct", "ok": False, "reason": f"battery {pct:.1%} < {self.min_battery_pct:.1%}"})
                elif voltage is not None and self.min_battery_v > 0 and voltage < self.min_battery_v:
                    allow = False
                    checks.append({"check": "battery_voltage", "ok": False, "reason": f"battery {voltage:.2f}V < {self.min_battery_v:.2f}V"})
                else:
                    checks.append({"check": "battery", "ok": True, "value": battery})
        else:
            allow = False
            checks.append({"check": "battery", "ok": False, "reason": "no BatteryState topic discovered (missing)"})

        joint = self.node.joint_state_cached()
        if joint.get("available"):
            if joint.get("age_s", 999) > self.max_joint_age_s:
                allow = False
                checks.append({"check": "joint_states_fresh", "ok": False, "reason": f"/joint_states stale: {joint['age_s']}s"})
            else:
                checks.append({"check": "joint_states_fresh", "ok": True, "age_s": joint["age_s"]})
        else:
            allow = False
            checks.append({"check": "joint_states", "ok": False, "reason": "no /joint_states evidence (missing)"})

        return {"allow": allow, "checks": checks}


# ──────────────────────────────────────────────────────────────────────────────
# Deterministic test runner
# ──────────────────────────────────────────────────────────────────────────────
TEST_CATALOG = {
    "graph_health": {
        "name": "ROS graph health",
        "description": "Check whether the graph contains useful nodes/topics and core robot signals.",
        "writes": False,
    },
    "diagnostics_health": {
        "name": "Standard diagnostics health",
        "description": "Inspect /diagnostics and fail on ERROR/STALE; WARN remains warning.",
        "writes": False,
    },
    "joint_states_health": {
        "name": "Joint-state stream health",
        "description": "Sample /joint_states and check freshness, rate, finite values and movement jitter evidence.",
        "writes": False,
    },
    "ros2_control_health": {
        "name": "ros2_control health",
        "description": "Inspect controller_manager controllers, hardware components and interfaces when available.",
        "writes": False,
    },
    "system_health": {
        "name": "Composite system health",
        "description": "Run graph, diagnostics, joint-state and ros2_control checks and aggregate the result.",
        "writes": False,
    },
}


class TestRunner:
    def __init__(self, node: Any, history: HistoryStore, max_diag_age_s: float = 5.0):
        self.node = node
        self.history = history
        self.max_diag_age_s = max_diag_age_s

    @staticmethod
    def _severity(result: str) -> int:
        return {"PASS": 0, "SKIP": 1, "WARN": 2, "FAIL": 3}.get(result, 3)

    def run(self, test_id: str) -> dict[str, Any]:
        if test_id not in TEST_CATALOG:
            return {"ok": False, "error": f"unknown test_id: {test_id}", "available": list(TEST_CATALOG)}

        started_iso = now_iso()
        started = time.monotonic()
        try:
            method = getattr(self, f"_test_{test_id}")
            result, summary, evidence = method()
        except Exception as exc:  # noqa: BLE001
            result, summary, evidence = (
                "FAIL",
                f"test exception: {type(exc).__name__}: {exc}",
                [],
            )
        duration_ms = int((time.monotonic() - started) * 1000)
        tr = TestResult(
            test_id=test_id,
            result=result,
            summary=summary,
            evidence=evidence,
            started_at=started_iso,
            duration_ms=duration_ms,
        )
        self.history.add(tr)
        # ok means "the test executed and produced a result", not "the test
        # passed". A FAIL is the most important kind of evidence, so it must
        # not be treated as a failed tool call downstream.
        return {"ok": True, **tr.as_dict()}

    def _test_graph_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        g = self.node.inspect_graph()
        topic_names = {x["name"] for x in g["topics"]}
        node_count = g["summary"]["nodes"]
        evidence = [
            asdict(Evidence("ros_graph", "node_count", node_count, now_iso())),
            asdict(Evidence("ros_graph", "topic_count", g["summary"]["topics"], now_iso())),
            asdict(Evidence("ros_graph", "has_joint_states", "/joint_states" in topic_names, now_iso())),
            asdict(Evidence("ros_graph", "has_diagnostics", "/diagnostics" in topic_names, now_iso())),
        ]
        if node_count <= 1:
            return "FAIL", "Only the harness node is visible; robot graph appears absent.", evidence
        if "/joint_states" not in topic_names:
            return "WARN", "ROS graph is alive, but /joint_states was not discovered.", evidence
        return "PASS", "ROS graph is alive and /joint_states is present.", evidence

    def _test_diagnostics_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        d = self.node.get_diagnostics(include_ok=True)
        evidence = [asdict(Evidence("/diagnostics", "summary", d, now_iso()))]
        if not d["available"]:
            return "SKIP", "No standard /diagnostics status has been received.", evidence
        age = d.get("last_array_age_s")
        if age is not None and age > self.max_diag_age_s:
            return "FAIL", f"/diagnostics is stale ({age}s; threshold {self.max_diag_age_s}s).", evidence
        statuses = d.get("statuses", [])
        levels = [x["level"] for x in statuses]
        if any(x >= DIAG_ERROR for x in levels):
            bad = [f"{x['name']}: {x.get('message') or ''}" for x in statuses if x["level"] >= DIAG_ERROR]
            return "FAIL", f"{len(bad)} diagnostic component(s) ERROR/STALE — " + "; ".join(bad[:5]), evidence
        if any(x == DIAG_WARN for x in levels):
            warns = [f"{x['name']}: {x.get('message') or ''}" for x in statuses if x["level"] == DIAG_WARN]
            return "WARN", f"{len(warns)} diagnostic component(s) WARN — " + "; ".join(warns[:5]), evidence
        return "PASS", "All received standard diagnostic statuses are OK.", evidence

    def _test_joint_states_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        s = self.node.sample_topic("/joint_states", duration_s=2.5, rate_hz=10.0)
        evidence = [asdict(Evidence("/joint_states", "sample", s, now_iso()))]
        if not s.get("ok"):
            return "FAIL", s.get("error", "joint state sampling failed"), evidence
        if s["samples"] < 3:
            return "WARN", f"Only {s['samples']} joint-state samples were received.", evidence
        if s["effective_rate_hz"] < 2.0:
            return "WARN", f"Joint-state effective sample rate is low ({s['effective_rate_hz']} Hz).", evidence

        # Look for non-finite or missing position data in latest sample.
        last = s.get("last", {})
        joints = last.get("_joints", {}) if isinstance(last, dict) else {}
        if not joints:
            return "WARN", "JointState was received but no named joints were found.", evidence
        missing = [name for name, val in joints.items() if val.get("position") is None]
        if missing:
            return "WARN", f"{len(missing)} joints have no position value.", evidence
        return "PASS", f"Joint-state stream healthy for {len(joints)} named joints.", evidence

    def _test_ros2_control_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        c = self.node.inspect_ros2_control()
        evidence = [asdict(Evidence("ros2_control", "inspection", c, now_iso()))]
        if not c.get("available"):
            return "SKIP", c.get("error", "ros2_control controller manager not discovered"), evidence

        controllers = (((c.get("controllers") or {}).get("data") or {}).get("controller") or [])
        hardware = (((c.get("hardware_components") or {}).get("data") or {}).get("component") or [])
        active_controllers = [x for x in controllers if str(x.get("state", "")).lower() == "active"]

        bad_hw = []
        for comp in hardware:
            state = comp.get("state") or {}
            label = str(state.get("label", "")).lower()
            if label and label != "active":
                bad_hw.append({"name": comp.get("name"), "state": state})

        if bad_hw:
            return "FAIL", f"{len(bad_hw)} ros2_control hardware component(s) are not active.", evidence
        if controllers and not active_controllers:
            return "WARN", "Controllers are loaded but none are active.", evidence
        return "PASS", f"ros2_control reachable; {len(active_controllers)} active controller(s).", evidence

    def _test_system_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        sub_ids = [
            "graph_health",
            "diagnostics_health",
            "joint_states_health",
            "ros2_control_health",
        ]
        sub_results = []
        for sid in sub_ids:
            # Call sub-test methods directly so composite run creates one DB record,
            # rather than recursively writing 5 history entries.
            result, summary, evidence = getattr(self, f"_test_{sid}")()
            sub_results.append({"test_id": sid, "result": result, "summary": summary, "evidence": evidence})

        worst = max(sub_results, key=lambda x: self._severity(x["result"]))["result"]
        if worst == "FAIL":
            overall = "FAIL"
        elif any(x["result"] == "WARN" for x in sub_results):
            overall = "WARN"
        else:
            overall = "PASS"

        summary = "; ".join(f"{x['test_id']}={x['result']}" for x in sub_results)
        return overall, summary, sub_results


# ──────────────────────────────────────────────────────────────────────────────
# ACTION capability registry
# ──────────────────────────────────────────────────────────────────────────────
def build_action_capabilities() -> dict[str, QueryCapability]:
    """Deterministic registry of supported ACTION intents.

    Only read-only tests and the software emergency stop are exposed. The AI
    may name one of these, but this registry (plus TEST_CATALOG) is what
    actually gates execution.
    """
    capabilities: dict[str, QueryCapability] = {
        "emergency_stop": QueryCapability(
            "emergency_stop",
            "Software stop: publish zero Twist and optionally call the configured Trigger E-stop service.",
            "emergency_stop",
        )
    }
    for test_id, spec in TEST_CATALOG.items():
        if spec.get("writes"):
            continue
        capabilities[f"run_test:{test_id}"] = QueryCapability(
            f"run_test:{test_id}",
            f"Run the deterministic {spec['name']} health test ({test_id}).",
            "run_test",
        )
    return capabilities

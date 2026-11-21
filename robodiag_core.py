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
import os
import re
import sqlite3
import threading
import time
from array import array
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from robodiag_jev import Evidence, QueryCapability, TestResult
from robodiag_rdcd import MISSING, TYPE_MISMATCH, Rdcd, capability_status

# DiagnosticStatus.level is a `byte` field; on ROS 2 Humble the class constants
# are `bytes` (b'\x00'..b'\x03'), so use plain ints internally.
DIAG_OK = 0
DIAG_WARN = 1
DIAG_ERROR = 2
DIAG_STALE = 3

# Tier-1 sensor-test thresholds. Module constants (rather than constructor
# parameters) to match the inline-threshold style already used by
# joint_states_health; lift into TestRunner.__init__ if per-robot tuning is
# ever needed.
IMU_QUATERNION_TOLERANCE = 0.15
IMU_ACCEL_G_MIN = 5.0   # m/s² — gravity plausibility floor
IMU_ACCEL_G_MAX = 15.0  # m/s² — gravity plausibility ceiling
IMU_MIN_RATE_HZ = 5.0
ODOM_MIN_RATE_HZ = 5.0
MAX_CMD_VEL_LINEAR = 2.0   # m/s
MAX_CMD_VEL_ANGULAR = 6.0  # rad/s

# Tier-2 cross-sensor consistency thresholds. Degrees are used for attitude so
# the numbers read naturally in test summaries; angular rates stay in rad/s.
ODOM_IMU_ROLL_PITCH_WARN_DEG = 5.0
ODOM_IMU_ROLL_PITCH_FAIL_DEG = 15.0
ODOM_IMU_ANGULAR_WARN_RAD_S = 0.05
ODOM_IMU_ANGULAR_FAIL_RAD_S = 0.20
CMD_VS_MOTION_CMD_THRESHOLD = 0.02     # m/s — below this a command counts as "at rest"
CMD_VS_MOTION_MOTION_THRESHOLD = 0.01  # m/s — below this odometry counts as "not moving"
CMD_VS_MOTION_RESPONSE_RATIO = 0.30    # observed/commanded below this → slip/stall warning

# Baseline drift thresholds, in standard deviations from the rolling mean.
BASELINE_MIN_SAMPLES = 5
BASELINE_WARN_SIGMA = 3.0
BASELINE_FAIL_SIGMA = 5.0
# Floor on the std used for z-scores, so a metric that was perfectly (or nearly)
# constant during warm-up does not turn tiny jitter into a huge sigma.
BASELINE_MIN_STD_REL = 0.01   # fraction of |mean|
BASELINE_MIN_STD_ABS = 1e-3


def _finite(value: Any) -> float | None:
    """Return ``value`` as a finite float, or None when missing/non-numeric/non-finite."""
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _covariance_meaningful(cov: Any) -> bool:
    """True when a covariance array contains at least one non-zero finite entry."""
    if not isinstance(cov, list) or not cov:
        return False
    for entry in cov:
        v = _finite(entry)
        if v is not None and v != 0.0:
            return True
    return False


def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
    """Convert a quaternion (x, y, z, w) to (roll, pitch, yaw) in radians."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _angle_diff_deg(a_deg: float, b_deg: float) -> float:
    """Signed shortest angular difference in degrees, wrapped to [-180, 180]."""
    return (a_deg - b_deg + 180.0) % 360.0 - 180.0


# ── Topic resolution from natural-language questions ──
TOPIC_RE = re.compile(r"/(?:[A-Za-z0-9_]+/)*[A-Za-z0-9_]+")


def extract_topic(text: str, default: str | None = None) -> str | None:
    match = TOPIC_RE.search(text)
    return match.group(0) if match else default


@dataclass(frozen=True)
class SensorAlias:
    label: str
    pattern: re.Pattern[str]
    capability: str       # RDCD capability name
    msg_type: str
    default_topic: str


SENSOR_ALIASES: tuple[SensorAlias, ...] = (
    SensorAlias("IMU", re.compile(r"imu|惯性|姿态|陀螺|加速度", re.I),
                 "imu", "sensor_msgs/msg/Imu", "/imu/data"),
    SensorAlias("odometry", re.compile(r"odom|里程", re.I),
                 "odometry", "nav_msgs/msg/Odometry", "/odom"),
    SensorAlias("joint_states", re.compile(r"joint|关节", re.I),
                 "joint_states", "sensor_msgs/msg/JointState", "/joint_states"),
    SensorAlias("battery", re.compile(r"battery|电池|电量", re.I),
                 "battery", "sensor_msgs/msg/BatteryState", "/battery_state"),
    SensorAlias("cmd_vel", re.compile(r"cmd_vel|速度指令|速度命令|velocity command", re.I),
                 "velocity_command", "geometry_msgs/msg/Twist", "/cmd_vel"),
)


def match_sensor_alias(text: str) -> SensorAlias | None:
    # Ignore any explicit /topic path so "/imu/data" does not widen the match.
    plain = TOPIC_RE.sub(" ", text)
    for alias in SENSOR_ALIASES:
        if alias.pattern.search(plain):
            return alias
    return None


HOST_RESOURCE_RE = re.compile(
    r"\bcpu\b|处理器|占用率|\b(memory|ram)\b|内存|\bload average\b|负载|\btemperature\b|\btemp\b|温度|发烫|\bdisk\b|磁盘|硬盘",
    re.IGNORECASE,
)


def host_resources() -> dict[str, Any]:
    """CPU / memory / temperature of the machine running the harness (Linux /proc, read-only)."""
    out: dict[str, Any] = {"ok": True, "scope": "host running robodiag-harness"}
    try:
        with open("/proc/loadavg", encoding="utf-8") as f:
            parts = f.read().split()
        cores = os.cpu_count() or 1
        out["cpu_cores"] = cores
        out["load_avg"] = {"1m": float(parts[0]), "5m": float(parts[1]), "15m": float(parts[2])}
        out["load_per_core_1m"] = round(float(parts[0]) / cores, 2)

        def _cpu_times() -> tuple[int, int]:
            with open("/proc/stat", encoding="utf-8") as f:
                vals = [int(v) for v in f.readline().split()[1:]]
            return sum(vals), vals[3] + (vals[4] if len(vals) > 4 else 0)

        t0, i0 = _cpu_times()
        time.sleep(0.5)
        t1, i1 = _cpu_times()
        if t1 > t0:
            out["cpu_percent"] = round(100.0 * (1 - (i1 - i0) / (t1 - t0)), 1)
    except (OSError, ValueError, IndexError) as exc:
        out["cpu_error"] = f"{type(exc).__name__}: {exc}"
    try:
        info: dict[str, int] = {}
        with open("/proc/meminfo", encoding="utf-8") as f:
            for line in f:
                k, _, v = line.partition(":")
                info[k] = int(v.split()[0])
        total, avail = info["MemTotal"], info["MemAvailable"]
        out["memory"] = {
            "total_mb": round(total / 1024),
            "available_mb": round(avail / 1024),
            "used_percent": round(100.0 * (1 - avail / total), 1),
        }
    except (OSError, ValueError, KeyError) as exc:
        out["memory_error"] = f"{type(exc).__name__}: {exc}"
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", encoding="utf-8") as f:
            out["temperature_c"] = round(int(f.read().strip()) / 1000.0, 1)
    except (OSError, ValueError):
        pass
    return out


def topics_for_alias(alias: SensorAlias, topics: list[dict[str, Any]], rdcd: Any = None) -> list[dict[str, Any]]:
    """Topics belonging to a sensor: declared in the RDCD, of its message type, or named like it."""
    declared = rdcd.get(alias.capability) if rdcd is not None else None
    ascii_kw = alias.default_topic.strip("/").split("/")[0].lower()
    out = []
    for t in topics:
        if (
            (declared is not None and t["name"] == declared.topic)
            or alias.msg_type in t.get("types", [])
            or ascii_kw in t["name"].lower()
        ):
            out.append(t)
    return out


def resolve_topic(text: str, node: Any, rdcd: Any = None) -> str | None:
    """Explicit /topic path first, otherwise infer it from a sensor name in the question."""
    explicit = extract_topic(text)
    if explicit:
        return explicit
    alias = match_sensor_alias(text)
    if alias is None:
        return None
    declared = rdcd.get(alias.capability) if rdcd is not None else None
    if declared is not None:
        return declared.topic
    topics = node.inspect_graph().get("topics", [])
    names = [t["name"] for t in topics]
    if alias.default_topic in names:
        return alias.default_topic
    typed = [t["name"] for t in topics if alias.msg_type in t.get("types", [])]
    return typed[0] if typed else None


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


def is_constant_zero(v: dict[str, Any]) -> bool:
    """True when every statistic is zero (e.g. unused covariance entries)."""
    for key in ("min", "max", "mean", "std", "range"):
        try:
            if float(v.get(key) or 0.0) != 0.0:
                return False
        except (TypeError, ValueError):
            return False
    return True


def sample_activity_summary(result: dict[str, Any]) -> str:
    """One-line /sample recap for the AI context, with a few varying fields.

    A bare "N samples @ X Hz" is not enough for a follow-up question such as
    "then what does it mean?"; include the most variable numeric fields so the
    answer can refer to actual values.
    """
    topic = result.get("topic", "?")
    if not result.get("ok"):
        return f"/sample {topic} → {result.get('error', 'failed')}"
    base = (
        f"/sample {topic} → {result.get('samples')} samples @ "
        f"{result.get('effective_rate_hz')} Hz"
    )
    stats = result.get("statistics", {}) or {}
    varying = []
    for path, v in stats.items():
        try:
            span = max(abs(float(v.get("range") or 0.0)), abs(float(v.get("std") or 0.0)))
        except (TypeError, ValueError):
            continue
        if span > 0:
            varying.append((span, path, v))
    varying.sort(key=lambda item: item[0], reverse=True)
    if varying:
        digest = "; ".join(
            f"{path} mean={v.get('mean')} range={v.get('range')}"
            for _, path, v in varying[:6]
        )
        base += f"; most variable: {digest}"
    return base


_VERIFY_WORD_RE = re.compile(r"\bverif\w*", re.IGNORECASE)
_VERIFY_CONTINUE_RE = re.compile(
    r"\b(?:continue|keep|more|further|again|go on|yes)\b", re.IGNORECASE
)


def is_verify_request(text: str) -> bool:
    """True when the operator is asking to continue diagnostic verification.

    Matches explicit forms such as ``/verify`` or ``continue verify`` without
    treating a fresh request like "verify the battery" as a continuation.
    """
    low = (text or "").strip().lower().rstrip("?!. ")
    if low in ("/verify", "verify", "verify all", "/verify all"):
        return True
    return bool(_VERIFY_WORD_RE.search(low)) and bool(_VERIFY_CONTINUE_RE.search(low))


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
# Rolling baseline (drift detection)
# ──────────────────────────────────────────────────────────────────────────────
class BaselineStore:
    """Per-metric rolling baseline for detecting drift from "normal".

    Uses Welford's online algorithm so only count/mean/M2/min/max are stored per
    metric — no raw value history grows without bound. A later observation can be
    compared against the baseline as a z-score (standard deviations from the
    rolling mean), which is what ``baseline_health`` reports.
    """

    def __init__(self, path: Path):
        self.path = path.expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        con = self._connect()
        try:
            with con:
                con.execute(
                    """
                    CREATE TABLE IF NOT EXISTS baseline_metrics (
                        metric TEXT PRIMARY KEY,
                        n INTEGER NOT NULL,
                        mean REAL NOT NULL,
                        m2 REAL NOT NULL,
                        min REAL NOT NULL,
                        max REAL NOT NULL,
                        updated_at TEXT NOT NULL
                    )
                    """
                )
        finally:
            con.close()

    def _connect(self) -> sqlite3.Connection:
        return sqlite3.connect(str(self.path), timeout=3.0)

    @staticmethod
    def _row_to_dict(row: tuple[Any, ...]) -> dict[str, Any]:
        metric, n, mean, m2, lo, hi, updated_at = row
        std = math.sqrt(m2 / (n - 1)) if n >= 2 else None
        return {
            "metric": metric,
            "n": n,
            "mean": round(mean, 6),
            "std": round(std, 6) if std is not None else None,
            "min": round(lo, 6),
            "max": round(hi, 6),
            "updated_at": updated_at,
        }

    def observe(self, metric: str, value: float) -> dict[str, Any]:
        """Merge one value into the baseline and return the updated row."""
        v = _finite(value)
        if v is None:
            raise ValueError(f"cannot baseline non-finite value {value!r}")
        with self._lock:
            con = self._connect()
            try:
                row = con.execute(
                    "SELECT metric, n, mean, m2, min, max, updated_at "
                    "FROM baseline_metrics WHERE metric = ?",
                    (metric,),
                ).fetchone()
                if row is None:
                    n, mean, m2, lo, hi = 1, v, 0.0, v, v
                else:
                    _, n, mean, m2, lo, hi, _ = row
                    n += 1
                    delta = v - mean
                    mean = mean + delta / n
                    delta2 = v - mean
                    m2 = m2 + delta * delta2
                    lo = min(lo, v)
                    hi = max(hi, v)
                updated_at = now_iso()
                with con:
                    con.execute(
                        "INSERT INTO baseline_metrics "
                        "(metric, n, mean, m2, min, max, updated_at) "
                        "VALUES (?, ?, ?, ?, ?, ?, ?) "
                        "ON CONFLICT(metric) DO UPDATE SET "
                        "n=excluded.n, mean=excluded.mean, m2=excluded.m2, "
                        "min=excluded.min, max=excluded.max, updated_at=excluded.updated_at",
                        (metric, n, mean, m2, lo, hi, updated_at),
                    )
            finally:
                con.close()
        return self._row_to_dict((metric, n, mean, m2, lo, hi, updated_at))

    def compare(self, metric: str, value: float) -> dict[str, Any]:
        """Compare a value against the baseline without updating it.

        Returns ``{metric, value, n, mean, std, sigma}`` where ``sigma`` is the
        absolute z-score (standard deviations from the mean, using a floored
        std; see ``BASELINE_MIN_STD_*``) and is None until at least two
        observations exist.
        """
        v = _finite(value)
        if v is None:
            raise ValueError(f"cannot compare non-finite value {value!r}")
        with self._lock:
            con = self._connect()
            try:
                row = con.execute(
                    "SELECT n, mean, m2 FROM baseline_metrics WHERE metric = ?",
                    (metric,),
                ).fetchone()
            finally:
                con.close()
        if row is None:
            return {"metric": metric, "value": round(v, 6), "n": 0, "mean": None, "std": None, "sigma": None}
        n, mean, m2 = row
        if n < 2:
            return {"metric": metric, "value": round(v, 6), "n": n, "mean": round(mean, 6), "std": None, "sigma": None}
        std = math.sqrt(m2 / (n - 1))
        effective_std = max(std, BASELINE_MIN_STD_REL * abs(mean), BASELINE_MIN_STD_ABS)
        sigma = abs(v - mean) / effective_std
        return {
            "metric": metric,
            "value": round(v, 6),
            "n": n,
            "mean": round(mean, 6),
            "std": round(std, 6),
            "sigma": round(sigma, 6),
        }

    def stats(self) -> list[dict[str, Any]]:
        """Return every baselined metric, ordered by most recent update."""
        with self._lock:
            con = self._connect()
            try:
                rows = con.execute(
                    "SELECT metric, n, mean, m2, min, max, updated_at "
                    "FROM baseline_metrics ORDER BY updated_at DESC"
                ).fetchall()
            finally:
                con.close()
        return [self._row_to_dict(r) for r in rows]

    def clear(self) -> None:
        with self._lock:
            con = self._connect()
            try:
                with con:
                    con.execute("DELETE FROM baseline_metrics")
            finally:
                con.close()


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
        rdcd: Rdcd | None = None,
    ):
        self.node = node
        self.min_battery_pct = min_battery_pct
        self.min_battery_v = min_battery_v
        self.max_diag_age_s = max_diag_age_s
        self.max_joint_age_s = max_joint_age_s
        self.max_battery_age_s = max_battery_age_s
        self.rdcd = rdcd

    def _requires(self, capability: str) -> bool:
        """Whether a capability belongs to the applicable safety evidence set.

        Generic mode (no RDCD) keeps the existing assumption that every
        evidence class is required. With an RDCD, only the capabilities the
        RDCD declares ``required_for_motion`` are required — falling back to
        "all declared capabilities" when the RDCD does not mark any. This lets
        e.g. a robot without battery telemetry avoid being blocked merely
        because no battery data exists.
        """
        if self.rdcd is None:
            return True
        required = self.rdcd.required_for_motion()
        if required:
            return capability in required
        return self.rdcd.has(capability)

    def check_for_motion(self) -> dict[str, Any]:
        """Fail closed: deny motion unless required evidence is present and healthy.

        Missing evidence (no /diagnostics, battery, or /joint_states) is treated
        as unsafe, never as "normal". This matches the design principle that
        "no data" must not permit motion. Capabilities that the active RDCD does
        not declare are simply not part of the required evidence set.
        """
        checks: list[dict[str, Any]] = []
        allow = True

        if self._requires("diagnostics"):
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

        if self._requires("battery"):
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

        if self._requires("joint_states"):
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
        "requires": "diagnostics",
    },
    "battery_health": {
        "name": "Battery health",
        "description": "Inspect the BatteryState topic and check percentage, voltage, freshness and validity.",
        "writes": False,
        "requires": "battery",
    },
    "joint_states_health": {
        "name": "Joint-state stream health",
        "description": "Sample /joint_states and check freshness, rate, finite values and movement jitter evidence.",
        "writes": False,
        "requires": "joint_states",
    },
    "ros2_control_health": {
        "name": "ros2_control health",
        "description": "Inspect controller_manager controllers, hardware components and interfaces when available.",
        "writes": False,
        "requires": "ros2_control",
    },
    "imu_health": {
        "name": "IMU health",
        "description": "Sample the IMU stream and check finite values, quaternion normalization, gravity plausibility and covariance population.",
        "writes": False,
        "requires": "imu",
    },
    "odometry_health": {
        "name": "Odometry health",
        "description": "Sample the odometry stream and check finite pose/twist values, quaternion normalization and covariance population.",
        "writes": False,
        "requires": "odometry",
    },
    "velocity_command_health": {
        "name": "Velocity command health",
        "description": "Observe the velocity-command topic and flag non-finite or out-of-bounds motion commands.",
        "writes": False,
        "requires": "velocity_command",
    },
    "declared_capability_check": {
        "name": "Declared capability check",
        "description": "Verify every RDCD-declared capability has its topic present with the declared message type.",
        "writes": False,
    },
    "system_health": {
        "name": "Composite system health",
        "description": "Run graph, diagnostics, battery, joint-state, ros2_control, IMU and odometry checks and aggregate the result.",
        "writes": False,
    },
    "baseline_health": {
        "name": "Baseline drift health",
        "description": "Compare current joint-state rate and IMU attitude/acceleration against the rolling baseline and flag drift beyond 3σ (WARN) / 5σ (FAIL). Builds the baseline on first runs.",
        "writes": False,
        "requires": "joint_states",
    },
    "odometry_vs_imu": {
        "name": "Odometry vs IMU consistency",
        "description": "Cross-check odometry pose/twist against IMU orientation/angular-velocity; disagreement beyond thresholds indicates sensor fault, calibration error or slip.",
        "writes": False,
        "requires": ["odometry", "imu"],
    },
    "cmd_vel_vs_motion": {
        "name": "Commanded vs observed motion",
        "description": "Compare velocity commands against observed odometry twist; flags commands that produced no motion (stalled/blocked) or far less motion than commanded (slip).",
        "writes": False,
        "requires": ["velocity_command", "odometry"],
    },
}


class TestRunner:
    def __init__(
        self,
        node: Any,
        history: HistoryStore,
        max_diag_age_s: float = 5.0,
        max_battery_age_s: float = 5.0,
        min_battery_pct: float = 0.10,
        rdcd: Rdcd | None = None,
        baseline: BaselineStore | None = None,
    ):
        self.node = node
        self.history = history
        self.max_diag_age_s = max_diag_age_s
        self.max_battery_age_s = max_battery_age_s
        self.min_battery_pct = min_battery_pct
        self.rdcd = rdcd
        self.baseline = baseline

    @staticmethod
    def _severity(result: str) -> int:
        return {"PASS": 0, "NOT_SUPPORTED": 0, "SKIP": 1, "WARN": 2, "FAIL": 3}.get(result, 3)

    def _requires(self, test_id: str) -> str | list[str] | None:
        return TEST_CATALOG[test_id].get("requires")

    def _not_supported(self, test_id: str) -> tuple[str, str, list[dict[str, Any]]] | None:
        """Return (result, summary, evidence) when a test's required capability
        is not declared by the active RDCD; None when the test is applicable.

        A test may require several capabilities (e.g. ``odometry_vs_imu`` needs
        both odometry and imu); it is NOT_SUPPORTED when any one is undeclared.
        """
        req = self._requires(test_id)
        if req is None or self.rdcd is None:
            return None
        required = [req] if isinstance(req, str) else list(req)
        missing = [c for c in required if not self.rdcd.has(c)]
        if missing:
            summary = (
                f"{', '.join(missing)} telemetry is not supported by "
                f"the {self.rdcd.name} RDCD."
            )
            return ("NOT_SUPPORTED", summary, [])
        return None

    def _capability_topic(self, capability: str, conventional: str) -> str | None:
        """Resolve the topic to probe for a capability.

        With an RDCD the declared topic wins (``run`` only reaches a test method
        after establishing the capability is declared, otherwise it returns
        NOT_SUPPORTED). Without an RDCD (generic mode) fall back to the
        conventional ROS topic name.
        """
        if self.rdcd is not None:
            spec = self.rdcd.get(capability)
            return spec.topic if spec is not None else None
        return conventional

    def run(self, test_id: str) -> dict[str, Any]:
        if test_id not in TEST_CATALOG:
            return {"ok": False, "error": f"unknown test_id: {test_id}", "available": list(TEST_CATALOG)}

        not_supported = self._not_supported(test_id)
        if not_supported is not None:
            result, summary, evidence = not_supported
            tr = TestResult(
                test_id=test_id,
                result=result,
                summary=summary,
                evidence=evidence,
                started_at=now_iso(),
                duration_ms=0,
            )
            self.history.add(tr)
            return {"ok": True, **tr.as_dict()}

        started_iso = now_iso()
        started = time.monotonic()
        try:
            # Plugin tests registered at runtime take precedence over built-in
            # ``_test_<id>`` methods, so third parties can add checks without
            # forking the harness.
            if test_id in _PLUGIN_TEST_FNS:
                result, summary, evidence = _PLUGIN_TEST_FNS[test_id](self)
            else:
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

    def _test_battery_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        b = self.node.battery_state()
        evidence = [asdict(Evidence("battery_state", "state", b, now_iso()))]
        if not b.get("available"):
            return "SKIP", "No BatteryState message has been received.", evidence

        age = b.get("age_s")
        if age is not None and age > self.max_battery_age_s:
            return "FAIL", f"BatteryState is stale ({age:.1f}s; threshold {self.max_battery_age_s:.1f}s).", evidence

        # _on_battery stores None for a non-finite voltage or a non-finite /
        # negative percentage, so a None here means the field was invalid.
        invalid = []
        if b.get("percentage") is None:
            invalid.append("percentage")
        if b.get("voltage") is None:
            invalid.append("voltage")
        if invalid:
            return "FAIL", "Invalid battery data: " + ", ".join(invalid) + ".", evidence

        pct = b["percentage"]
        voltage = b["voltage"]
        if pct < self.min_battery_pct:
            return "WARN", f"Battery low: {pct:.0%} below {self.min_battery_pct:.0%} ({voltage:.1f} V).", evidence
        if b.get("present") is False:
            return "WARN", "Battery reports present=False (no battery detected).", evidence

        return "PASS", f"BatteryState healthy: {pct:.0%} ({voltage:.1f} V).", evidence

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

    def _test_imu_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        topic = self._capability_topic("imu", "/imu/data")
        declared = self.rdcd is not None
        s = self.node.sample_topic(topic, duration_s=2.0, rate_hz=20.0)
        evidence = [asdict(Evidence(topic, "sample", s, now_iso()))]
        if not s.get("ok"):
            if declared:
                return "FAIL", f"Declared IMU stream unavailable: {s.get('error')}", evidence
            return "SKIP", f"No IMU stream discovered ({s.get('error')}); skipped.", evidence

        last = s.get("last", {}) if isinstance(s.get("last"), dict) else {}
        orient = last.get("orientation") or {}
        angular = last.get("angular_velocity") or {}
        accel = last.get("linear_acceleration") or {}
        if not orient and not angular and not accel:
            if declared:
                return "FAIL", "IMU topic returned no IMU fields (message-type mismatch?).", evidence
            return "SKIP", "Sampled topic does not carry IMU fields; skipped.", evidence

        fields = {
            "orientation.x": orient.get("x"),
            "orientation.y": orient.get("y"),
            "orientation.z": orient.get("z"),
            "orientation.w": orient.get("w"),
            "angular_velocity.x": angular.get("x"),
            "angular_velocity.y": angular.get("y"),
            "angular_velocity.z": angular.get("z"),
            "linear_acceleration.x": accel.get("x"),
            "linear_acceleration.y": accel.get("y"),
            "linear_acceleration.z": accel.get("z"),
        }
        nonfinite = sorted(k for k, v in fields.items() if _finite(v) is None)
        if nonfinite:
            return "FAIL", "IMU has non-finite fields: " + ", ".join(nonfinite) + ".", evidence

        qx, qy, qz, qw = (_finite(orient.get(k)) for k in ("x", "y", "z", "w"))
        qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        quat_dev = abs(qnorm - 1.0)

        ax, ay, az = (_finite(accel.get(k)) for k in ("x", "y", "z"))
        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)

        cov_ok = _covariance_meaningful(last.get("orientation_covariance"))
        rate = float(s.get("effective_rate_hz", 0.0) or 0.0)

        if quat_dev > 3.0 * IMU_QUATERNION_TOLERANCE:
            return "FAIL", f"IMU quaternion is degenerate (norm={qnorm:.3f}).", evidence

        problems = []
        if quat_dev > IMU_QUATERNION_TOLERANCE:
            problems.append(f"quaternion norm deviates from 1 (norm={qnorm:.3f})")
        if not (IMU_ACCEL_G_MIN <= accel_mag <= IMU_ACCEL_G_MAX):
            problems.append(f"linear acceleration magnitude {accel_mag:.1f} m/s² outside [{IMU_ACCEL_G_MIN}, {IMU_ACCEL_G_MAX}]")
        if not cov_ok:
            problems.append("orientation covariance is unpopulated (all zero/absent)")
        if rate < IMU_MIN_RATE_HZ:
            problems.append(f"effective publish rate {rate:.1f} Hz below {IMU_MIN_RATE_HZ} Hz")

        if problems:
            return "WARN", "; ".join(problems) + ".", evidence
        return "PASS", f"IMU stream healthy ({rate:.0f} Hz, |accel|={accel_mag:.1f} m/s²).", evidence

    def _test_odometry_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        topic = self._capability_topic("odometry", "/odom")
        declared = self.rdcd is not None
        s = self.node.sample_topic(topic, duration_s=2.0, rate_hz=20.0)
        evidence = [asdict(Evidence(topic, "sample", s, now_iso()))]
        if not s.get("ok"):
            if declared:
                return "FAIL", f"Declared odometry stream unavailable: {s.get('error')}", evidence
            return "SKIP", f"No odometry stream discovered ({s.get('error')}); skipped.", evidence

        last = s.get("last", {}) if isinstance(s.get("last"), dict) else {}
        pose = last.get("pose") or {}
        pose_pose = pose.get("pose") or {}
        position = pose_pose.get("position") or {}
        orient = pose_pose.get("orientation") or {}
        twist = (last.get("twist") or {}).get("twist") or {}
        linear = twist.get("linear") or {}
        angular = twist.get("angular") or {}
        if not position and not orient and not linear and not angular:
            if declared:
                return "FAIL", "Odometry topic returned no odometry fields (message-type mismatch?).", evidence
            return "SKIP", "Sampled topic does not carry odometry fields; skipped.", evidence

        fields = {
            "position.x": position.get("x"),
            "position.y": position.get("y"),
            "position.z": position.get("z"),
            "orientation.x": orient.get("x"),
            "orientation.y": orient.get("y"),
            "orientation.z": orient.get("z"),
            "orientation.w": orient.get("w"),
            "twist.linear.x": linear.get("x"),
            "twist.linear.y": linear.get("y"),
            "twist.linear.z": linear.get("z"),
            "twist.angular.x": angular.get("x"),
            "twist.angular.y": angular.get("y"),
            "twist.angular.z": angular.get("z"),
        }
        nonfinite = sorted(k for k, v in fields.items() if _finite(v) is None)
        if nonfinite:
            return "FAIL", "Odometry has non-finite fields: " + ", ".join(nonfinite) + ".", evidence

        qx, qy, qz, qw = (_finite(orient.get(k)) for k in ("x", "y", "z", "w"))
        qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        quat_dev = abs(qnorm - 1.0)
        pose_cov_ok = _covariance_meaningful(pose.get("covariance"))
        rate = float(s.get("effective_rate_hz", 0.0) or 0.0)

        if quat_dev > 3.0 * IMU_QUATERNION_TOLERANCE:
            return "FAIL", f"Odometry orientation quaternion is degenerate (norm={qnorm:.3f}).", evidence

        problems = []
        if quat_dev > IMU_QUATERNION_TOLERANCE:
            problems.append(f"orientation quaternion norm deviates from 1 (norm={qnorm:.3f})")
        if not pose_cov_ok:
            problems.append("pose covariance is unpopulated (all zero/absent)")
        if rate < ODOM_MIN_RATE_HZ:
            problems.append(f"effective publish rate {rate:.1f} Hz below {ODOM_MIN_RATE_HZ} Hz")

        if problems:
            return "WARN", "; ".join(problems) + ".", evidence
        return "PASS", f"Odometry stream healthy ({rate:.0f} Hz).", evidence

    def _test_velocity_command_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        topic = self._capability_topic("velocity_command", "/cmd_vel")
        declared = self.rdcd is not None
        s = self.node.sample_topic(topic, duration_s=1.5, rate_hz=20.0)
        evidence = [asdict(Evidence(topic, "sample", s, now_iso()))]
        if not s.get("ok"):
            err = s.get("error", "")
            if "no samples" in err or "no message" in err:
                return "PASS", "No velocity commands observed; robot is at rest.", evidence
            if declared:
                return "FAIL", f"Declared velocity-command stream unavailable: {err}", evidence
            return "SKIP", f"No usable velocity-command topic ({err}); skipped.", evidence

        last = s.get("last", {}) if isinstance(s.get("last"), dict) else {}
        linear = last.get("linear") or {}
        angular = last.get("angular") or {}
        if not linear and not angular:
            if declared:
                return "FAIL", "Velocity-command topic returned no Twist fields (message-type mismatch?).", evidence
            return "SKIP", "Sampled topic does not carry Twist fields; skipped.", evidence

        nonfinite = []
        for k in ("x", "y", "z"):
            if _finite(linear.get(k)) is None:
                nonfinite.append(f"linear.{k}")
            if _finite(angular.get(k)) is None:
                nonfinite.append(f"angular.{k}")
        if nonfinite:
            return "FAIL", "Velocity command has non-finite fields: " + ", ".join(nonfinite) + ".", evidence

        stats = s.get("statistics") or {}

        def _peak(path: str) -> float:
            v = stats.get(path)
            if not isinstance(v, dict):
                return 0.0
            lo = _finite(v.get("min"))
            hi = _finite(v.get("max"))
            return max(abs(lo or 0.0), abs(hi or 0.0))

        linear_peak = max(_peak(f"linear.{k}") for k in ("x", "y", "z"))
        angular_peak = max(_peak(f"angular.{k}") for k in ("x", "y", "z"))

        problems = []
        if linear_peak > MAX_CMD_VEL_LINEAR:
            problems.append(f"linear velocity command peak {linear_peak:.2f} m/s exceeds {MAX_CMD_VEL_LINEAR:.1f} m/s")
        if angular_peak > MAX_CMD_VEL_ANGULAR:
            problems.append(f"angular velocity command peak {angular_peak:.2f} rad/s exceeds {MAX_CMD_VEL_ANGULAR:.1f} rad/s")

        if problems:
            return "WARN", "; ".join(problems) + ".", evidence
        return "PASS", f"Velocity commands within bounds (peak {linear_peak:.2f} m/s, {angular_peak:.2f} rad/s).", evidence

    @staticmethod
    def _peak_suffix(stats: dict[str, Any], *suffixes: str) -> float:
        """Peak absolute value of any statistic whose path ends with a suffix.

        Suffix matching keeps this robust to message nesting (e.g. odometry's
        ``twist.twist.linear.x`` vs a bare Twist's ``linear.x``).
        """
        best = 0.0
        for path, v in stats.items():
            if not isinstance(v, dict) or not any(path.endswith(s) for s in suffixes):
                continue
            lo = _finite(v.get("min"))
            hi = _finite(v.get("max"))
            best = max(best, abs(lo or 0.0), abs(hi or 0.0))
        return best

    def _test_baseline_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        if self.baseline is None:
            return "SKIP", "Baseline store is not configured.", []

        metrics: list[tuple[str, float]] = []
        evidence: list[dict[str, Any]] = []

        js_topic = self._capability_topic("joint_states", "/joint_states")
        js = self.node.sample_topic(js_topic, duration_s=2.5, rate_hz=10.0)
        evidence.append(asdict(Evidence(js_topic, "sample", js, now_iso())))
        if not js.get("ok"):
            return "FAIL", f"Joint-state sampling failed: {js.get('error')}", evidence
        if js.get("samples", 0) >= 2:
            metrics.append(("joint_states.rate_hz", float(js.get("effective_rate_hz", 0.0) or 0.0)))

        if self.rdcd is None or self.rdcd.has("imu"):
            imu_topic = self._capability_topic("imu", "/imu/data")
            if imu_topic:
                s = self.node.sample_topic(imu_topic, duration_s=2.0, rate_hz=20.0)
                evidence.append(asdict(Evidence(imu_topic, "sample", s, now_iso())))
                if s.get("ok"):
                    last = s.get("last", {}) if isinstance(s.get("last"), dict) else {}
                    accel = last.get("linear_acceleration") or {}
                    axyz = [_finite(accel.get(k)) for k in ("x", "y", "z")]
                    if all(v is not None for v in axyz):
                        metrics.append(("imu.accel_mag", math.sqrt(sum(v * v for v in axyz))))
                    orient = last.get("orientation") or {}
                    q = [_finite(orient.get(k)) for k in ("x", "y", "z", "w")]
                    if all(v is not None for v in q):
                        metrics.append(("imu.quat_norm", math.sqrt(sum(v * v for v in q))))

        if self.rdcd is None or self.rdcd.has("odometry"):
            odom_topic = self._capability_topic("odometry", "/odom")
            if odom_topic:
                s = self.node.sample_topic(odom_topic, duration_s=2.0, rate_hz=10.0)
                evidence.append(asdict(Evidence(odom_topic, "sample", s, now_iso())))
                if s.get("ok"):
                    last = s.get("last", {}) if isinstance(s.get("last"), dict) else {}
                    pose = ((last.get("pose") or {}).get("pose") or {})
                    orient = pose.get("orientation") or {}
                    q = [_finite(orient.get(k)) for k in ("x", "y", "z", "w")]
                    if all(v is not None for v in q):
                        metrics.append(("odometry.quat_norm", math.sqrt(sum(v * v for v in q))))

        if not metrics:
            return "SKIP", "No baselinable metrics could be measured.", evidence

        # Baselines are per robot so different RDCDs never share statistics.
        scope = self.rdcd.robot_id if self.rdcd is not None else "generic"
        metrics = [(f"{scope}:{m}", v) for m, v in metrics]

        comparisons = [self.baseline.compare(m, v) for m, v in metrics]
        # Learn only from values that are not drifting; otherwise a persistent
        # fault would gradually become the "normal" baseline.
        for (m, v), c in zip(metrics, comparisons):
            if c["sigma"] is None or c["sigma"] <= BASELINE_WARN_SIGMA:
                self.baseline.observe(m, v)
        evidence.extend(
            asdict(Evidence("baseline", c["metric"], c, now_iso())) for c in comparisons
        )

        established = [
            c for c in comparisons
            if c["n"] >= BASELINE_MIN_SAMPLES and c["sigma"] is not None
        ]
        if not established:
            min_n = min(c["n"] for c in comparisons)
            behind = ", ".join(
                f"{c['metric']} n={c['n']}"
                for c in comparisons
                if c["n"] == min_n
            )
            remaining = max(0, BASELINE_MIN_SAMPLES - min_n)
            return (
                "SKIP",
                f"Baseline not established; recorded {len(metrics)} metric(s) this run "
                f"(minimum prior count {min_n}/{BASELINE_MIN_SAMPLES}: {behind}). "
                f"Run baseline_health {remaining} more time(s) to establish.",
                evidence,
            )

        worst = max(established, key=lambda c: c["sigma"])
        sigma = worst["sigma"]
        detail = ", ".join(
            f"{c['metric']}={c['sigma']}σ"
            for c in sorted(established, key=lambda c: -c["sigma"])[:6]
        )
        if sigma > BASELINE_FAIL_SIGMA:
            return "FAIL", f"Baseline drift: {worst['metric']} at {sigma:.1f}σ ({detail}).", evidence
        if sigma > BASELINE_WARN_SIGMA:
            return "WARN", f"Baseline drift: {worst['metric']} at {sigma:.1f}σ ({detail}).", evidence
        return "PASS", f"No significant drift (max {sigma:.1f}σ across {len(established)} metric(s)).", evidence

    def _test_odometry_vs_imu(self) -> tuple[str, str, list[dict[str, Any]]]:
        declared = self.rdcd is not None
        odom_topic = self._capability_topic("odometry", "/odom")
        imu_topic = self._capability_topic("imu", "/imu/data")
        odom_s = self.node.sample_topic(odom_topic, duration_s=2.0, rate_hz=10.0)
        imu_s = self.node.sample_topic(imu_topic, duration_s=2.0, rate_hz=10.0)
        evidence = [
            asdict(Evidence(odom_topic, "sample", odom_s, now_iso())),
            asdict(Evidence(imu_topic, "sample", imu_s, now_iso())),
        ]
        if not odom_s.get("ok") or not imu_s.get("ok"):
            if declared:
                which = "Odometry" if not odom_s.get("ok") else "IMU"
                return "FAIL", f"{which} stream unavailable in the declared RDCD.", evidence
            return "SKIP", "Odometry/IMU cross-check needs both streams; not discovered in generic mode.", evidence

        odom_last = odom_s.get("last", {}) if isinstance(odom_s.get("last"), dict) else {}
        imu_last = imu_s.get("last", {}) if isinstance(imu_s.get("last"), dict) else {}
        odom_orient = (((odom_last.get("pose") or {}).get("pose") or {}).get("orientation")) or {}
        imu_orient = imu_last.get("orientation") or {}

        q_odom = [_finite(odom_orient.get(k)) for k in ("x", "y", "z", "w")]
        q_imu = [_finite(imu_orient.get(k)) for k in ("x", "y", "z", "w")]
        if any(v is None for v in q_odom) or any(v is None for v in q_imu):
            if declared:
                return "FAIL", "Missing orientation quaternion fields in odometry/IMU.", evidence
            return "SKIP", "Sampled topics do not carry odometry/IMU orientation fields; skipped.", evidence

        r_odom, p_odom, _ = _quat_to_rpy(*q_odom)
        r_imu, p_imu, _ = _quat_to_rpy(*q_imu)
        roll_dev = abs(_angle_diff_deg(math.degrees(r_odom), math.degrees(r_imu)))
        pitch_dev = abs(_angle_diff_deg(math.degrees(p_odom), math.degrees(p_imu)))

        odom_ang = (odom_last.get("twist") or {}).get("twist") or {}
        odom_ang_z = _finite((odom_ang.get("angular") or {}).get("z"))
        imu_ang_z = _finite((imu_last.get("angular_velocity") or {}).get("z"))
        ang_dev = abs(odom_ang_z - imu_ang_z) if (odom_ang_z is not None and imu_ang_z is not None) else None

        problems: list[str] = []
        hard = False
        if roll_dev > ODOM_IMU_ROLL_PITCH_FAIL_DEG:
            hard = True
            problems.append(f"roll mismatch {roll_dev:.1f}°")
        elif roll_dev > ODOM_IMU_ROLL_PITCH_WARN_DEG:
            problems.append(f"roll mismatch {roll_dev:.1f}°")
        if pitch_dev > ODOM_IMU_ROLL_PITCH_FAIL_DEG:
            hard = True
            problems.append(f"pitch mismatch {pitch_dev:.1f}°")
        elif pitch_dev > ODOM_IMU_ROLL_PITCH_WARN_DEG:
            problems.append(f"pitch mismatch {pitch_dev:.1f}°")
        if ang_dev is not None:
            if ang_dev > ODOM_IMU_ANGULAR_FAIL_RAD_S:
                hard = True
                problems.append(f"angular-rate mismatch {ang_dev:.3f} rad/s")
            elif ang_dev > ODOM_IMU_ANGULAR_WARN_RAD_S:
                problems.append(f"angular-rate mismatch {ang_dev:.3f} rad/s")

        if hard:
            return "FAIL", "; ".join(problems) + ".", evidence
        if problems:
            return "WARN", "; ".join(problems) + ".", evidence
        return "PASS", f"Odometry and IMU agree (roll Δ{roll_dev:.1f}°, pitch Δ{pitch_dev:.1f}°).", evidence

    def _test_cmd_vel_vs_motion(self) -> tuple[str, str, list[dict[str, Any]]]:
        cmd_topic = self._capability_topic("velocity_command", "/cmd_vel")
        odom_topic = self._capability_topic("odometry", "/odom")
        cmd_s = self.node.sample_topic(cmd_topic, duration_s=1.5, rate_hz=20.0)
        odom_s = self.node.sample_topic(odom_topic, duration_s=1.5, rate_hz=20.0)
        evidence = [
            asdict(Evidence(cmd_topic, "sample", cmd_s, now_iso())),
            asdict(Evidence(odom_topic, "sample", odom_s, now_iso())),
        ]
        if not cmd_s.get("ok"):
            err = cmd_s.get("error", "")
            if "no samples" in err or "no message" in err:
                return "PASS", "No velocity commands observed; robot is at rest.", evidence
            return "FAIL", f"Velocity-command stream unavailable: {err}", evidence

        cmd_stats = cmd_s.get("statistics") or {}
        cmd_linear = self._peak_suffix(cmd_stats, "linear.x", "linear.y", "linear.z")
        cmd_angular = self._peak_suffix(cmd_stats, "angular.x", "angular.y", "angular.z")
        cmd_peak = max(cmd_linear, cmd_angular)

        if cmd_peak < CMD_VS_MOTION_CMD_THRESHOLD:
            return "PASS", "Robot at rest (no meaningful velocity command).", evidence

        if not odom_s.get("ok"):
            return "WARN", f"Command peak {cmd_peak:.2f} but odometry unavailable to confirm motion.", evidence

        odom_stats = odom_s.get("statistics") or {}
        odom_linear = self._peak_suffix(odom_stats, "linear.x", "linear.y", "linear.z")
        odom_angular = self._peak_suffix(odom_stats, "angular.x", "angular.y", "angular.z")
        odom_peak = max(odom_linear, odom_angular)

        if odom_peak < CMD_VS_MOTION_MOTION_THRESHOLD:
            return "FAIL", (
                f"Velocity command peak {cmd_peak:.2f} produced no odometry motion "
                "(stalled/blocked?)."
            ), evidence

        if cmd_linear > CMD_VS_MOTION_CMD_THRESHOLD:
            ratio = odom_linear / cmd_linear
            if ratio < CMD_VS_MOTION_RESPONSE_RATIO:
                return "WARN", (
                    f"Observed motion ({odom_linear:.3f} m/s) far below commanded "
                    f"({cmd_linear:.3f} m/s) — possible slip/stall."
                ), evidence

        return "PASS", f"Commanded motion ({cmd_peak:.2f}) matched by odometry ({odom_peak:.2f}).", evidence

    def _test_declared_capability_check(self) -> tuple[str, str, list[dict[str, Any]]]:
        if self.rdcd is None:
            return "SKIP", "No RDCD loaded; declared-capability validation is not applicable.", []

        graph = self.node.inspect_graph() if self.node is not None else {}
        topics = graph.get("topics", []) if isinstance(graph, dict) else []
        topic_types = {t.get("name"): t.get("types", []) for t in topics if isinstance(t, dict)}

        rows = []
        for cap_name in self.rdcd.declared():
            spec = self.rdcd.get(cap_name)
            status = capability_status(self.rdcd, cap_name, topic_types)
            rows.append({
                "capability": cap_name,
                "topic": spec.topic,
                "expected_type": spec.message_type,
                "status": status,
            })
        evidence = [asdict(Evidence("rdcd", self.rdcd.robot_id, {"capabilities": rows}, now_iso()))]

        bad = [r for r in rows if r["status"] in (MISSING, TYPE_MISMATCH)]
        if bad:
            detail = "; ".join(f"{r['capability']}={r['status']}" for r in bad)
            return "FAIL", f"{len(bad)} declared capability(s) missing/type-mismatched: {detail}", evidence
        return "PASS", f"All {len(rows)} declared capabilities are present with the declared message type.", evidence

    def _test_system_health(self) -> tuple[str, str, list[dict[str, Any]]]:
        sub_ids = [
            "graph_health",
            "diagnostics_health",
            "battery_health",
            "joint_states_health",
            "ros2_control_health",
            "imu_health",
            "odometry_health",
            "odometry_vs_imu",
            "cmd_vel_vs_motion",
        ]
        sub_results = []
        for sid in sub_ids:
            # Call sub-test methods directly so composite run creates one DB record,
            # rather than recursively writing 5 history entries.
            not_supported = self._not_supported(sid)
            if not_supported is not None:
                result, summary, evidence = not_supported
            else:
                result, summary, evidence = getattr(self, f"_test_{sid}")()
            sub_results.append({"test_id": sid, "result": result, "summary": summary, "evidence": evidence})

        # Unsupported capabilities (NOT_SUPPORTED) must never drag down the
        # composite result; only applicable tests participate in aggregation.
        applicable = [x for x in sub_results if x["result"] != "NOT_SUPPORTED"]
        if not applicable:
            overall = "PASS"
        else:
            worst = max(applicable, key=lambda x: self._severity(x["result"]))["result"]
            if worst == "FAIL":
                overall = "FAIL"
            elif any(x["result"] == "WARN" for x in applicable):
                overall = "WARN"
            else:
                overall = "PASS"

        summary = "; ".join(
            f"{x['test_id']}={'N/A' if x['result'] == 'NOT_SUPPORTED' else x['result']}"
            for x in sub_results
        )
        return overall, summary, sub_results


# ──────────────────────────────────────────────────────────────────────────────
# Plugin test registry
# ──────────────────────────────────────────────────────────────────────────────
_PLUGIN_TEST_GROUP = "robodiag.tests"

# test_id → callable(TestRunner) -> (result, summary, evidence). Plugin tests are
# resolved in TestRunner.run before the built-in ``_test_<id>`` methods, so third
# parties can add deterministic checks without forking the harness.
_PLUGIN_TEST_FNS: dict[str, Any] = {}


def register_test(
    test_id: str,
    name: str,
    description: str,
    fn: Any,
    requires: str | list[str] | None = None,
    writes: bool = False,
) -> None:
    """Register a plugin health test into the deterministic catalog.

    ``fn`` is called as ``fn(TestRunner)`` and must return
    ``(result, summary, evidence)`` with the same contract as the built-in
    ``_test_*`` methods.
    """
    spec: dict[str, Any] = {
        "name": name,
        "description": description,
        "writes": bool(writes),
    }
    if requires:
        spec["requires"] = requires
    TEST_CATALOG[test_id] = spec
    _PLUGIN_TEST_FNS[test_id] = fn


def load_entry_point_tests(group: str = _PLUGIN_TEST_GROUP) -> list[str]:
    """Discover plugin tests declared via ``robodiag.tests`` entry points.

    An entry point must resolve to a callable taking ``TestRunner`` and returning
    ``(result, summary, evidence)``. Optional attributes on the callable carry
    metadata: ``name``, ``description``, ``requires``, ``writes``.
    """
    try:
        from importlib import metadata as _ilm
    except ImportError:  # pragma: no cover - importlib.metadata is stdlib 3.8+
        return []
    try:
        eps = _ilm.entry_points(group=group)
    except TypeError:  # pragma: no cover - Python < 3.10 returns a dict-like
        eps = _ilm.entry_points().get(group, [])
    loaded: list[str] = []
    for ep in eps:
        try:
            fn = ep.load()
        except Exception:  # noqa: BLE001
            continue
        if not callable(fn):
            continue
        register_test(
            ep.name,
            getattr(fn, "name", ep.name),
            getattr(fn, "description", ""),
            fn,
            getattr(fn, "requires", None),
            getattr(fn, "writes", False),
        )
        loaded.append(ep.name)
    return loaded


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

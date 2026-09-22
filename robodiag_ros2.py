#!/usr/bin/env python3
"""
RoboDiag ROS 2 Diagnostic Harness v0.2
======================================

Target: ROS 2 Humble (Python 3.10), but mostly distro-agnostic.

Capabilities
------------
- Inspect ROS graph: nodes / topics / services
- Collect standard /diagnostics (diagnostic_msgs/DiagnosticArray)
- Inspect ros2_control through controller_manager services when available
- Dynamically snapshot/sample arbitrary ROS topics without hard-coding message types
- Produce deterministic health tests and persist test history in SQLite
- Safety Gate for motion-related extensions
- Emergency stop fallback: publish zero geometry_msgs/Twist to a configurable cmd_vel topic
- Optional Trigger-based E-stop service
- Optional DeepSeek/OpenAI-compatible diagnostic Agent with function calling
- Optional Jev (System One) next-tool router: Jev decides which evidence to
  collect next, the LLM writes the final diagnosis

Design rule
-----------
The LLM may request evidence and tests, but deterministic code decides whether a
write/motion operation is permitted. The default version does NOT include any test
that intentionally moves the robot.

Examples
--------
    source /opt/ros/humble/setup.bash
    source ~/robot_ws/install/setup.bash
    python3 robodiag_ros2.py

    python3 robodiag_ros2.py --cmd-vel-topic /cmd_vel
    python3 robodiag_ros2.py --controller-manager /controller_manager
    python3 robodiag_ros2.py --estop-service /emergency_stop

REPL commands
-------------
    /help
    /graph
    /diagnostics
    /control
    /check
    /topic /joint_states
    /sample /joint_states 5 5
    /tests
    /run joint_states_health
    /history 10
    /stop
    /quit

Environment variables
---------------------
    DEEPSEEK_API_KEY
    DEEPSEEK_BASE_URL=https://api.deepseek.com
    DEEPSEEK_MODEL=deepseek-chat

    ROBODIAG_MIN_BATTERY_PCT=0.10
    ROBODIAG_MIN_BATTERY_V=0.0
    ROBODIAG_MAX_BATTERY_AGE_S=5.0
    ROBODIAG_MAX_DIAG_AGE_S=5.0
    ROBODIAG_MAX_JOINT_STATE_AGE_S=1.0
    ROBODIAG_DB=~/.robodiag_ros2.db
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

from robodiag_jev import (
    JEV_ACTIONS,
    LANGUAGE_NAMES,
    QUERY_CAPABILITIES,
    DiagnosticState,
    Evidence,
    JevClient,
    JevDecision,
    JevError,
    JevRouter,
    RequestMode,
    RequestRoute,
    RequestRouter,
    TYPESAFE_DEFAULT_BASE_URL,
    TYPESAFE_DEFAULT_MODEL,
    TestResult,
    ToolCall,
    detect_language,
)

from robodiag_core import (
    DIAG_ERROR,
    DIAG_OK,
    DIAG_WARN,
    HistoryStore,
    SafetyGate,
    TEST_CATALOG,
    TestRunner,
    _jsonable,
    _result_ok,
    build_action_capabilities,
    compact,
    now_iso,
    stats_from_samples,
)

# ──────────────────────────────────────────────────────────────────────────────
# REPL line editing / completion
#
# prompt_toolkit provides a live completion menu (dropdown) for slash commands.
# readline is kept as a fallback so Console.input() still handles arrow keys and
# history when prompt_toolkit is missing or stdin is not a TTY.
try:
    from prompt_toolkit import PromptSession
    from prompt_toolkit.auto_suggest import AutoSuggestFromHistory
    from prompt_toolkit.completion import Completer, Completion
    from prompt_toolkit.formatted_text import ANSI as _ANSI
    from prompt_toolkit.history import FileHistory

    _HAVE_PROMPT_TOOLKIT = True
except ImportError:  # pragma: no cover - optional dependency
    _HAVE_PROMPT_TOOLKIT = False

try:
    import atexit
    import readline

    _HISTORY_FILE = Path.home() / ".robodiag_ros2_readline_history"
    try:
        readline.read_history_file(_HISTORY_FILE)
    except OSError:
        pass
    readline.set_history_length(1000)

    def _save_readline_history() -> None:
        try:
            readline.write_history_file(_HISTORY_FILE)
        except OSError:
            pass

    atexit.register(_save_readline_history)
except ImportError:  # pragma: no cover - readline missing on some Python builds
    pass

# ──────────────────────────────────────────────────────────────────────────────
# Friendly dependency checks
# ──────────────────────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
except ImportError as exc:
    print("RoboDiag requires ROS 2 Python (rclpy).")
    print("Source your ROS environment first, e.g. source /opt/ros/humble/setup.bash")
    raise SystemExit(2) from exc

try:
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import BatteryState, JointState, Imu
    from std_srvs.srv import Trigger
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    print(f"Missing ROS package dependency: {exc}")
    print("For Humble, install common packages such as diagnostic-msgs, sensor-msgs, geometry-msgs.")
    raise SystemExit(2) from exc

try:
    from rich.console import Console
    from rich.live import Live
    from rich.markdown import Markdown
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
except ImportError as exc:
    print("Missing Python package 'rich'. Install with: python3 -m pip install --user rich")
    raise SystemExit(2) from exc

try:
    from openai import OpenAI
except ImportError:
    OpenAI = None  # AI remains optional


VERSION = "0.2.0"

console = Console()

BANNER = [
    "██████╗  ██████╗ ██████╗  ██████╗ ██████╗ ██╗ █████╗  ██████╗ ",
    "██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗██╔══██╗██║██╔══██╗██╔════╝ ",
    "██████╔╝██║   ██║██████╔╝██║   ██║██║  ██║██║███████║██║  ███╗",
    "██╔══██╗██║   ██║██╔══██╗██║   ██║██║  ██║██║██╔══██║██║   ██║",
    "██║  ██║╚██████╔╝██████╔╝╚██████╔╝██████╔╝██║██║  ██║╚██████╔╝",
    "╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝ ╚═════╝ ╚═╝╚═╝  ╚═╝ ╚═════╝ ",
]
PALETTE = ["#00E5FF", "#00B8D9", "#7C4DFF", "#B388FF", "#FF4081", "#FF80AB"]


def banner_text(frame: int, width: int | None = None) -> Text:
    pad = " "
    if width:
        bw = max(len(line) for line in BANNER)
        pad = " " * max(0, (width - bw) // 2)
    t = Text()
    for li, line in enumerate(BANNER):
        t.append(pad)
        for ci, ch in enumerate(line):
            if ch == " ":
                t.append(ch)
            else:
                t.append(ch, style=PALETTE[(ci + li * 2 + frame) % len(PALETTE)])
        if li < len(BANNER) - 1:
            t.append("\n")
    return t


def play_banner() -> None:
    console.clear()
    w, h = console.width, console.height
    top_pad = max(0, (h - len(BANNER) - 7) // 2)
    console.print("\n" * top_pad, end="")
    with Live(banner_text(0, w), console=console, refresh_per_second=30) as live:
        for frame in range(1, 12):
            time.sleep(0.045)
            live.update(banner_text(frame, w))
    console.print()
    console.print(
        Text("ROS 2 Diagnostic Harness", style="bold #B388FF")
        .append(f"  ·  v{VERSION}", style="dim"),
        justify="center",
    )
    console.print(
        Text("Graph → Evidence → Test → Diagnosis", style="dim italic"),
        justify="center",
    )
    console.print()


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
def load_env(path: Path) -> dict[str, str]:
    cfg: dict[str, str] = {}
    if not path.exists():
        return cfg
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                cfg[k.strip()] = v.strip()
    except OSError:
        pass
    return cfg


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _diag_level(value: Any) -> int:
    """DiagnosticStatus.level is a `byte` field; ROS 2 Humble maps it to bytes."""
    if isinstance(value, (bytes, bytearray)):
        return value[0] if value else DIAG_OK
    return int(value)


def to_builtin_message(msg: Any) -> dict[str, Any]:
    """Convert ROS message to JSON-safe dict and add useful derived fields."""
    raw = _jsonable(message_to_ordereddict(msg, truncate_length=256))

    # Make JointState evidence human-readable by joint name rather than only indices.
    if isinstance(msg, JointState):
        joints: dict[str, dict[str, float | None]] = {}
        for i, name in enumerate(msg.name):
            joints[name] = {
                "position": float(msg.position[i]) if i < len(msg.position) else None,
                "velocity": float(msg.velocity[i]) if i < len(msg.velocity) else None,
                "effort": float(msg.effort[i]) if i < len(msg.effort) else None,
            }
        raw["_joints"] = joints

    # Quaternion -> roll/pitch/yaw helps diagnosis without relying on the LLM to calculate.
    if isinstance(msg, Imu):
        q = msg.orientation
        x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        raw["_orientation_rpy_deg"] = {
            "roll": round(math.degrees(roll), 4),
            "pitch": round(math.degrees(pitch), 4),
            "yaw": round(math.degrees(yaw), 4),
        }

    return raw


# ──────────────────────────────────────────────────────────────────────────────
# Evidence / test model
#
# Evidence, TestResult, ToolCall and DiagnosticState now live in robodiag_jev.py
# (imported above) so the diagnostic state model is shared with the router layer
# and has no ROS or rich dependency.
# ──────────────────────────────────────────────────────────────────────────────


# ──────────────────────────────────────────────────────────────────────────────
# ROS 2 probe node
# ──────────────────────────────────────────────────────────────────────────────
class _TopicCollector:
    """Message sink for a long-lived ad-hoc topic subscription.

    rclpy's MultiThreadedExecutor corrupts its wait set when a subscription is
    destroyed from a non-executor thread while the executor is spinning; after
    that, newly created subscriptions never receive messages. The harness keeps
    one subscription per probed topic for the whole session and routes deliveries
    through this collector instead of create/destroy churn.
    """

    def __init__(self, type_name: str) -> None:
        self.type_name = type_name
        self._lock = threading.Lock()
        self._collecting = False
        self._sample = False
        self._start = 0.0
        self._duration = 0.0
        self._min_interval = 0.0
        self._last_kept = 0.0
        self._sink: list[dict[str, Any]] = []
        self._first: dict[str, Any] | None = None
        self._first_at: str | None = None
        self._error: str | None = None
        self.first_event = threading.Event()
        self.done_event = threading.Event()

    def on_message(self, msg: Any) -> None:
        with self._lock:
            if not self._collecting:
                return
            now = time.monotonic()
            if now - self._start > self._duration:
                self.done_event.set()
                return
            if now - self._last_kept < self._min_interval:
                return
            self._last_kept = now
            try:
                data = to_builtin_message(msg)
            except Exception as exc:  # noqa: BLE001
                self._error = f"message conversion failed: {type(exc).__name__}: {exc}"
                self.first_event.set()
                self.done_event.set()
                return
            if self._first is None:
                self._first = data
                self._first_at = now_iso()
                self.first_event.set()
            if self._sample:
                self._sink.append(data)

    def start(self, duration_s: float, rate_hz: float, sample: bool) -> None:
        with self._lock:
            self._collecting = True
            self._sample = sample
            self._start = time.monotonic()
            self._duration = duration_s
            self._min_interval = 0.0 if rate_hz <= 0 else 1.0 / rate_hz
            self._last_kept = 0.0
            self._sink = []
            self._first = None
            self._first_at = None
            self._error = None
            self.first_event.clear()
            self.done_event.clear()

    def finish(self) -> tuple[list[dict[str, Any]], dict[str, Any] | None, str | None, str | None]:
        with self._lock:
            self._collecting = False
            return list(self._sink), self._first, self._first_at, self._error


class HarnessNode(Node):
    def __init__(
        self,
        controller_manager: str,
        cmd_vel_topic: str,
        estop_service: str | None,
    ):
        super().__init__("robodiag_harness")
        self.controller_manager = controller_manager.rstrip("/") or "/controller_manager"
        self.cmd_vel_topic = cmd_vel_topic
        self.estop_service = estop_service

        self._diag_lock = threading.Lock()
        self._diag_status: dict[str, dict[str, Any]] = {}
        self._last_diag_array_monotonic: float | None = None

        self._battery_lock = threading.Lock()
        self._last_battery: dict[str, Any] | None = None
        self._last_battery_monotonic: float | None = None

        self._joint_lock = threading.Lock()
        self._last_joint_state: dict[str, Any] | None = None
        self._last_joint_monotonic: float | None = None

        # Long-lived subscriptions for ad-hoc topic probing; see _dynamic_subscription().
        self._dyn_lock = threading.Lock()
        self._dyn_subs: dict[str, tuple[_TopicCollector, str, Any]] = {}

        # Best-effort subscriber is compatible with both common best-effort sensor
        # publishers and reliable publishers.
        self.sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.diag_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, self.sensor_qos
        )

        self.battery_sub = None
        self.joint_sub = None
        self._ensure_common_subscriptions()

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

    def _topic_type_map(self) -> dict[str, list[str]]:
        return {name: types for name, types in self.get_topic_names_and_types()}

    def _ensure_common_subscriptions(self) -> None:
        topics = self._topic_type_map()
        if self.battery_sub is None:
            candidates = [
                name
                for name, types in topics.items()
                if "sensor_msgs/msg/BatteryState" in types
            ]
            if candidates:
                # Prefer the conventional name if present.
                topic = "/battery_state" if "/battery_state" in candidates else candidates[0]
                self.battery_sub = self.create_subscription(
                    BatteryState, topic, self._on_battery, self.sensor_qos
                )
        if self.joint_sub is None and "/joint_states" in topics:
            if "sensor_msgs/msg/JointState" in topics["/joint_states"]:
                self.joint_sub = self.create_subscription(
                    JointState, "/joint_states", self._on_joint_state, self.sensor_qos
                )

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        now_mono = time.monotonic()
        with self._diag_lock:
            self._last_diag_array_monotonic = now_mono
            for st in msg.status:
                self._diag_status[st.name] = {
                    "level": _diag_level(st.level),
                    "level_name": self.diag_level_name(_diag_level(st.level)),
                    "name": st.name,
                    "message": st.message,
                    "hardware_id": st.hardware_id,
                    "values": {kv.key: kv.value for kv in st.values},
                    "received_at": now_iso(),
                    "received_monotonic": now_mono,
                }

    def _on_battery(self, msg: BatteryState) -> None:
        with self._battery_lock:
            pct = float(msg.percentage)
            voltage = float(msg.voltage)
            self._last_battery = {
                "voltage": None if not math.isfinite(voltage) else voltage,
                "percentage": None if not math.isfinite(pct) or pct < 0 else pct,
                "power_supply_status": int(msg.power_supply_status),
                "present": bool(msg.present),
                "received_at": now_iso(),
            }
            self._last_battery_monotonic = time.monotonic()

    def _on_joint_state(self, msg: JointState) -> None:
        with self._joint_lock:
            self._last_joint_state = to_builtin_message(msg)
            self._last_joint_monotonic = time.monotonic()

    @staticmethod
    def diag_level_name(level: int) -> str:
        return {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}.get(level, f"UNKNOWN({level})")

    # ── Graph ────────────────────────────────────────────────────────────────
    def inspect_graph(self) -> dict[str, Any]:
        self._ensure_common_subscriptions()
        nodes = [
            {"name": name, "namespace": ns}
            for name, ns in self.get_node_names_and_namespaces()
        ]
        topics = [
            {"name": name, "types": types}
            for name, types in self.get_topic_names_and_types()
        ]
        services = [
            {"name": name, "types": types}
            for name, types in self.get_service_names_and_types()
        ]
        return {
            "ok": True,
            "summary": {
                "nodes": len(nodes),
                "topics": len(topics),
                "services": len(services),
            },
            "nodes": sorted(nodes, key=lambda x: (x["namespace"], x["name"])),
            "topics": sorted(topics, key=lambda x: x["name"]),
            "services": sorted(services, key=lambda x: x["name"]),
        }

    # ── Diagnostics ──────────────────────────────────────────────────────────
    def get_diagnostics(self, include_ok: bool = True) -> dict[str, Any]:
        with self._diag_lock:
            rows = []
            now_mono = time.monotonic()
            for item in self._diag_status.values():
                row = dict(item)
                age = now_mono - float(row.pop("received_monotonic", now_mono))
                row["age_s"] = round(age, 3)
                if include_ok or row["level"] != DIAG_OK:
                    rows.append(row)
            rows.sort(key=lambda x: (-x["level"], x["name"]))

            overall = "UNKNOWN"
            if rows or self._diag_status:
                levels = [int(x["level"]) for x in self._diag_status.values()]
                worst = max(levels) if levels else -1
                overall = self.diag_level_name(worst)

            age = (
                None
                if self._last_diag_array_monotonic is None
                else round(now_mono - self._last_diag_array_monotonic, 3)
            )

        return {
            "ok": True,
            "available": bool(self._diag_status),
            "overall": overall,
            "last_array_age_s": age,
            "count": len(self._diag_status),
            "statuses": rows,
        }

    # ── Dynamic topic snapshot / sampling ───────────────────────────────────
    def resolve_topic_type(self, topic: str) -> tuple[type | None, str | None, str | None]:
        mapping = self._topic_type_map()
        if topic not in mapping:
            return None, None, f"topic not found: {topic}"
        types = mapping[topic]
        if not types:
            return None, None, f"topic has no discovered type: {topic}"
        if len(types) > 1:
            # Multiple types on one name is unusual and unsafe for automatic selection.
            return None, None, f"topic has multiple types {types}; refusing to guess"
        type_name = types[0]
        try:
            msg_type = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            return None, type_name, f"cannot import {type_name}: {exc}"
        return msg_type, type_name, None

    def _dynamic_subscription(
        self, topic: str
    ) -> tuple[_TopicCollector | None, str | None, str | None]:
        """Return a reusable collector for `topic`, creating its subscription once.

        Subscriptions are never destroyed while the executor spins: doing so from
        the REPL thread corrupts rclpy's MultiThreadedExecutor wait set, after
        which further subscriptions silently receive nothing.
        """
        with self._dyn_lock:
            entry = self._dyn_subs.get(topic)
            if entry is not None:
                return entry[0], entry[1], None
            msg_type, type_name, err = self.resolve_topic_type(topic)
            if err:
                return None, None, err
            collector = _TopicCollector(type_name)
            sub = self.create_subscription(msg_type, topic, collector.on_message, self.sensor_qos)
            self._dyn_subs[topic] = (collector, type_name, sub)
            return collector, type_name, None

    def topic_snapshot(self, topic: str, timeout_s: float = 3.0) -> dict[str, Any]:
        timeout_s = max(0.2, min(float(timeout_s), 15.0))
        collector, type_name, err = self._dynamic_subscription(topic)
        if err:
            return {"ok": False, "error": err, "topic": topic}
        assert collector is not None and type_name is not None

        collector.start(timeout_s, rate_hz=0.0, sample=False)
        try:
            got = collector.first_event.wait(timeout_s)
        finally:
            _, first, first_at, conv_err = collector.finish()

        if conv_err:
            return {"ok": False, "topic": topic, "type": type_name, "error": conv_err}
        if not got or first is None:
            return {
                "ok": False,
                "topic": topic,
                "type": type_name,
                "error": f"no message within {timeout_s:.1f}s",
            }
        return {
            "ok": True,
            "topic": topic,
            "type": type_name,
            "received_at": first_at or now_iso(),
            "message": first,
        }

    def sample_topic(
        self,
        topic: str,
        duration_s: float = 5.0,
        rate_hz: float = 5.0,
    ) -> dict[str, Any]:
        duration_s = max(0.5, min(float(duration_s), 30.0))
        rate_hz = max(0.2, min(float(rate_hz), 50.0))
        collector, type_name, err = self._dynamic_subscription(topic)
        if err:
            return {"ok": False, "error": err, "topic": topic}
        assert collector is not None and type_name is not None

        collector.start(duration_s, rate_hz=rate_hz, sample=True)
        try:
            # Sampling is based on wall time; the executor receives callbacks in background.
            collector.done_event.wait(duration_s + 0.15)
        finally:
            rows, _, _, conv_err = collector.finish()

        if conv_err:
            return {"ok": False, "topic": topic, "type": type_name, "error": conv_err}
        if not rows:
            return {
                "ok": False,
                "topic": topic,
                "type": type_name,
                "error": f"no samples received during {duration_s:.1f}s",
            }

        stats = stats_from_samples(rows)
        return {
            "ok": True,
            "topic": topic,
            "type": type_name,
            "duration_s": duration_s,
            "requested_rate_hz": rate_hz,
            "samples": len(rows),
            "effective_rate_hz": round(len(rows) / duration_s, 3),
            "statistics": stats,
            "first": rows[0],
            "last": rows[-1],
        }

    # ── ros2_control ─────────────────────────────────────────────────────────
    def _wait_future(self, future: Any, timeout_s: float) -> tuple[bool, Any]:
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if future.done():
                try:
                    return True, future.result()
                except Exception as exc:  # noqa: BLE001
                    return False, f"{type(exc).__name__}: {exc}"
            time.sleep(0.03)
        return False, "timeout"

    def inspect_ros2_control(self, timeout_s: float = 2.0) -> dict[str, Any]:
        try:
            from controller_manager_msgs.srv import (
                ListControllers,
                ListHardwareComponents,
                ListHardwareInterfaces,
            )
        except ImportError as exc:
            return {
                "ok": False,
                "available": False,
                "error": f"controller_manager_msgs not installed: {exc}",
            }

        base = self.controller_manager
        specs = [
            ("controllers", ListControllers, f"{base}/list_controllers"),
            ("hardware_components", ListHardwareComponents, f"{base}/list_hardware_components"),
            ("hardware_interfaces", ListHardwareInterfaces, f"{base}/list_hardware_interfaces"),
        ]
        out: dict[str, Any] = {"ok": True, "available": False, "controller_manager": base}
        clients = []
        try:
            for key, srv_type, srv_name in specs:
                cli = self.create_client(srv_type, srv_name)
                clients.append(cli)
                if not cli.wait_for_service(timeout_sec=0.5):
                    out[key] = {"available": False, "service": srv_name}
                    continue
                out["available"] = True
                fut = cli.call_async(srv_type.Request())
                ok, resp = self._wait_future(fut, timeout_s)
                if not ok:
                    out[key] = {"available": True, "error": resp}
                    continue
                out[key] = {"available": True, "data": _jsonable(message_to_ordereddict(resp))}
        finally:
            for cli in clients:
                try:
                    self.destroy_client(cli)
                except Exception:
                    pass

        if not out["available"]:
            out["ok"] = False
            out["error"] = f"controller manager services not found under {base}"
        return out

    # ── Common cached health evidence ────────────────────────────────────────
    def battery_state(self) -> dict[str, Any]:
        self._ensure_common_subscriptions()
        with self._battery_lock:
            if self._last_battery is None:
                return {"available": False}
            age = (
                None
                if self._last_battery_monotonic is None
                else time.monotonic() - self._last_battery_monotonic
            )
            return {"available": True, "age_s": round(age or 0.0, 3), **self._last_battery}

    def joint_state_cached(self) -> dict[str, Any]:
        self._ensure_common_subscriptions()
        with self._joint_lock:
            if self._last_joint_state is None:
                return {"available": False}
            age = (
                None
                if self._last_joint_monotonic is None
                else time.monotonic() - self._last_joint_monotonic
            )
            return {
                "available": True,
                "age_s": round(age or 0.0, 3),
                "message": self._last_joint_state,
            }

    # ── Emergency stop fallback ──────────────────────────────────────────────
    def emergency_stop(self) -> dict[str, Any]:
        """
        Best-effort software stop, NOT a certified hardware E-stop.

        1) publish zero Twist multiple times;
        2) optionally call a user-configured std_srvs/Trigger service.
        """
        actions: list[dict[str, Any]] = []
        zero = Twist()
        try:
            for _ in range(5):
                self.cmd_vel_pub.publish(zero)
                time.sleep(0.04)
            actions.append({"type": "cmd_vel_zero", "topic": self.cmd_vel_topic, "ok": True})
        except Exception as exc:  # noqa: BLE001
            actions.append({"type": "cmd_vel_zero", "topic": self.cmd_vel_topic, "ok": False, "error": str(exc)})

        if self.estop_service:
            cli = self.create_client(Trigger, self.estop_service)
            try:
                if cli.wait_for_service(timeout_sec=0.8):
                    fut = cli.call_async(Trigger.Request())
                    ok, resp = self._wait_future(fut, 2.0)
                    if ok:
                        actions.append(
                            {
                                "type": "trigger_estop",
                                "service": self.estop_service,
                                "ok": bool(resp.success),
                                "message": resp.message,
                            }
                        )
                    else:
                        actions.append({"type": "trigger_estop", "service": self.estop_service, "ok": False, "error": resp})
                else:
                    actions.append({"type": "trigger_estop", "service": self.estop_service, "ok": False, "error": "service unavailable"})
            finally:
                try:
                    self.destroy_client(cli)
                except Exception:
                    pass

        return {
            "ok": any(a.get("ok") for a in actions),
            "warning": "software stop only; this is not a certified hardware emergency stop",
            "actions": actions,
        }


# ──────────────────────────────────────────────────────────────────────────────
# LLM tools / Agent
# ──────────────────────────────────────────────────────────────────────────────
TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "inspect_graph",
            "description": "Inspect the current ROS 2 graph: nodes, topics, services and their types. Read-only.",
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_diagnostics",
            "description": "Read cached standard ROS /diagnostics statuses. Use this early for component-level faults.",
            "parameters": {
                "type": "object",
                "properties": {"include_ok": {"type": "boolean", "description": "Include OK entries; default false."}},
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "inspect_ros2_control",
            "description": "Inspect ros2_control controllers, hardware components and interfaces using controller_manager services. Read-only; may be unavailable on robots that do not use ros2_control.",
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "get_topic_snapshot",
            "description": "Read one message from any discovered ROS 2 topic by dynamically loading its message type.",
            "parameters": {
                "type": "object",
                "properties": {
                    "topic": {"type": "string", "description": "Absolute topic name, e.g. /joint_states or /imu/data"},
                    "timeout_s": {"type": "number", "description": "Wait timeout, default 3 seconds, max 15"},
                },
                "required": ["topic"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "sample_topic",
            "description": "Sample any discovered ROS 2 topic over a time window and calculate numeric min/max/mean/std/range. Use this for jitter, drift, dropouts and intermittent faults.",
            "parameters": {
                "type": "object",
                "properties": {
                    "topic": {"type": "string"},
                    "duration_s": {"type": "number", "description": "0.5-30 seconds, default 5"},
                    "rate_hz": {"type": "number", "description": "0.2-50 Hz, default 5"},
                },
                "required": ["topic"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "list_tests",
            "description": "List deterministic RoboDiag test cases. v0.2 tests are read-only and never intentionally move the robot.",
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "run_test",
            "description": "Run one deterministic RoboDiag health test by test_id. v0.2 test catalog is read-only.",
            "parameters": {
                "type": "object",
                "properties": {"test_id": {"type": "string", "enum": list(TEST_CATALOG.keys())}},
                "required": ["test_id"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "query_history",
            "description": "Read recent deterministic RoboDiag test history from local SQLite.",
            "parameters": {
                "type": "object",
                "properties": {"limit": {"type": "integer", "description": "1-100, default 10"}},
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "check_motion_safety",
            "description": "Run deterministic motion Safety Gate checks. This does not move the robot.",
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
    {
        "type": "function",
        "function": {
            "name": "emergency_stop",
            "description": "Immediately request a software stop. This publishes zero Twist and optionally invokes the configured Trigger E-stop service. Call without confirmation if a dangerous motion is observed. It is not a certified hardware E-stop.",
            "parameters": {"type": "object", "properties": {}, "required": []},
        },
    },
]


SYSTEM_PROMPT = f"""You are RoboDiag ROS 2 Diagnostic Agent v{VERSION}, running in a terminal next to the robot.

Your job: use real ROS 2 evidence to locate problems in the robot's software, communication, sensors, controllers and actuator chain.

Principles:
1. Gather evidence first, then reason. Do not invent the current hardware state from the robot model or general assumptions.
2. Prefer standard ROS capabilities: ROS graph, /diagnostics, /joint_states, IMU/Battery topics, ros2_control.
3. When suspecting jitter/drift/intermittent faults, use sample_topic instead of looking at a single frame only.
4. Conclusions must distinguish: measured evidence / inference / still-missing information. Use "suspected" or "likely" for causes; say "confirmed" only when a deterministic test established it.
5. When a tool returns ok=false or unavailable, explicitly acknowledge that the evidence is unavailable; do not interpret "no data" as "normal".
6. emergency_stop is a software-level fallback, not a physical E-stop; if dangerous motion is observed you may call it immediately without user confirmation.
7. v0.2 has no diagnostic test that actively drives joints; do not claim that you made the robot perform a motion test.
8. Answer in the same language as the operator (if the language is unclear, reply in English), organized as: symptom → evidence → assessment → possible causes → recommended next checks. Keep it engineering-focused and concise.
"""


class AgentRuntime:
    def __init__(
        self,
        node: HarnessNode,
        runner: TestRunner,
        history: HistoryStore,
        gate: SafetyGate,
    ):
        self.node = node
        self.runner = runner
        self.history = history
        self.gate = gate

    def execute(self, name: str, args: dict[str, Any]) -> str:
        try:
            if name == "inspect_graph":
                return compact(self.node.inspect_graph())
            if name == "get_diagnostics":
                return compact(self.node.get_diagnostics(bool(args.get("include_ok", False))))
            if name == "inspect_ros2_control":
                return compact(self.node.inspect_ros2_control())
            if name == "get_topic_snapshot":
                return compact(self.node.topic_snapshot(args.get("topic", ""), float(args.get("timeout_s", 3.0))))
            if name == "sample_topic":
                return compact(
                    self.node.sample_topic(
                        args.get("topic", ""),
                        float(args.get("duration_s", 5.0)),
                        float(args.get("rate_hz", 5.0)),
                    )
                )
            if name == "list_tests":
                return compact({"ok": True, "tests": TEST_CATALOG})
            if name == "run_test":
                return compact(self.runner.run(str(args.get("test_id", ""))))
            if name == "query_history":
                return compact({"ok": True, "items": self.history.recent(int(args.get("limit", 10)))})
            if name == "check_motion_safety":
                return compact({"ok": True, **self.gate.check_for_motion()})
            if name == "emergency_stop":
                return compact(self.node.emergency_stop())
            return compact({"ok": False, "error": f"unknown tool: {name}"})
        except Exception as exc:  # tools never throw to LLM
            return compact({"ok": False, "error": f"{type(exc).__name__}: {exc}"})


def parse_tool_args(raw: str) -> dict[str, Any] | None:
    try:
        value = json.loads(raw or "{}")
        return value if isinstance(value, dict) else None
    except ValueError:
        return None


def agent_turn(runtime: AgentRuntime, client: Any, model: str, messages: list[dict[str, Any]]) -> list[dict[str, Any]]:
    messages = list(messages)
    for _ in range(10):
        with console.status("[bold #00E5FF]Diagnosing…", spinner="dots"):
            try:
                resp = client.chat.completions.create(
                    model=model,
                    messages=messages,
                    tools=TOOLS,
                    temperature=0.2,
                )
            except KeyboardInterrupt:
                console.print("[yellow]Agent call interrupted.[/yellow]")
                return messages
            except Exception as exc:  # noqa: BLE001
                console.print(f"[red]LLM call failed: {type(exc).__name__}: {exc}[/red]")
                return messages

        msg = resp.choices[0].message
        record: dict[str, Any] = {"role": "assistant", "content": msg.content or ""}
        if msg.tool_calls:
            record["tool_calls"] = [tc.model_dump() for tc in msg.tool_calls]
        messages.append(record)

        if not msg.tool_calls:
            console.print(
                Panel(
                    Markdown(msg.content or "(empty)"),
                    border_style="#7C4DFF",
                    title=f"[bold]🩺 RoboDiag ROS 2 v{VERSION}[/bold]",
                )
            )
            return messages

        for tc in msg.tool_calls:
            name = tc.function.name
            args = parse_tool_args(tc.function.arguments)
            if args is None:
                result = compact({"ok": False, "error": "tool arguments are not valid JSON"})
            else:
                with console.status(f"[bold #00E5FF]⚡ {name} {args}", spinner="bouncingBall"):
                    result = runtime.execute(name, args)
            preview = result if len(result) <= 420 else result[:420] + " …"
            console.print(Text(f"  ⚡ {name} → {preview}", style="dim"))
            messages.append({"role": "tool", "tool_call_id": tc.id, "content": result})

    console.print("[yellow]Reached the tool-call round limit; stopping this agent turn.[/yellow]")
    return messages


# ──────────────────────────────────────────────────────────────────────────────
# Jev — System One decision router (TypeSafe AI)
#
# Jev is not a text-generating LLM. It takes a state (symptom + evidence) and a
# typed question, and returns a structured decision with calibrated
# probabilities. RoboDiag uses Jev as the "System 1" router that decides which
# deterministic tool to run next; once Jev says "finalize", the collected
# evidence is handed to the LLM (System 2) for the written diagnosis.
#
#   Jev decides what to investigate.
#   Deterministic tools decide what is true.
#   The LLM explains what it means.
#   The Safety Gate decides what is allowed.
#
# API: POST https://api.typesafe.ai/v1/systemone  (Authorization: Bearer <key>)
# See https://docs.typesafe.ai/api
# ──────────────────────────────────────────────────────────────────────────────
# Topic names Jev is allowed to choose from when a snapshot/sample is needed.
_JEV_TOPIC_HINTS = (
    "joint", "imu", "battery", "diagnostic", "cmd_vel", "camera", "scan",
    "odom", "wheel", "velocity", "position", "status", "effort", "temp",
    "current", "voltage", "foot", "leg", "motor",
)
_JEV_MAX_TOPIC_OPTIONS = 24
_JEV_MAX_ROUNDS = 8
_EXPLAINER_EVIDENCE_CHARS = 3000

# Deterministic gates around Jev's routing proposals (see run_jev_diagnosis).
_JEV_MIN_EVIDENCE_ITEMS = 1   # finalize is withheld until this many successful calls
_JEV_MAX_CALLS_PER_TOOL = 3   # per-tool budget inside one diagnosis
_JEV_NONE_TOPIC = "none_of_the_above"  # always offered so Jev is never forced into a bad topic

def _resolve_jev_settings(
    api_key_override: str | None = None,
    base_url_override: str | None = None,
    model_override: str | None = None,
) -> tuple[str, str, str]:
    """Resolve Jev settings with the same precedence as the LLM settings:
    CLI flag > environment variable > ~/.robodiag.env > built-in default."""
    env_file = load_env(Path.home() / ".robodiag.env")
    api_key = api_key_override or os.environ.get("TYPESAFE_API_KEY") or env_file.get("TYPESAFE_API_KEY", "")
    base_url = base_url_override or os.environ.get("TYPESAFE_BASE_URL") or env_file.get("TYPESAFE_BASE_URL", TYPESAFE_DEFAULT_BASE_URL)
    model = model_override or os.environ.get("TYPESAFE_MODEL") or env_file.get("TYPESAFE_MODEL", TYPESAFE_DEFAULT_MODEL)
    return api_key, base_url, model


def build_jev_router(
    api_key_override: str | None = None,
    base_url_override: str | None = None,
    model_override: str | None = None,
) -> tuple[JevRouter | None, str]:
    api_key, base_url, model = _resolve_jev_settings(api_key_override, base_url_override, model_override)
    if not api_key or "your-api-key" in api_key:
        return None, model
    try:
        return JevRouter(JevClient(api_key=api_key, base_url=base_url, model=model), TEST_CATALOG), model
    except Exception as exc:  # noqa: BLE001
        console.print(f"[yellow]Jev router initialization failed; disabled: {type(exc).__name__}: {exc}[/yellow]")
        return None, model


def build_request_router(
    api_key_override: str | None = None,
    base_url_override: str | None = None,
    model_override: str | None = None,
) -> tuple[RequestRouter | None, str]:
    api_key, base_url, model = _resolve_jev_settings(api_key_override, base_url_override, model_override)
    if not api_key or "your-api-key" in api_key:
        return None, model
    try:
        client = JevClient(api_key=api_key, base_url=base_url, model=model)
        router = RequestRouter(client, QUERY_CAPABILITIES, build_action_capabilities())
        return router, model
    except Exception as exc:  # noqa: BLE001
        console.print(f"[yellow]Request router initialization failed; disabled: {type(exc).__name__}: {exc}[/yellow]")
        return None, model


EXPLAINER_SYSTEM_PROMPT = f"""You are the RoboDiag ROS 2 Diagnostic Agent v{VERSION} explainer (System 2).

The evidence-gathering phase is complete. You receive the operator's symptom
and the structured evidence already collected by deterministic tools. Do not
request more tools.

Write the final diagnosis with this exact structure:
1. Symptom — restate the reported problem.
2. Evidence — the measured findings that matter, each attributed to its source.
3. Assessment — what the evidence directly shows, separating measured facts
   from inference.
4. Possible causes — ranked suspected/likely causes. Use "suspected" or
   "likely"; never claim a "root cause" unless a deterministic test has
   explicitly confirmed it.
5. Confirmed findings — only facts established by a deterministic test result.
   If there are none, say so explicitly.
6. Recommended next checks — concrete, ordered steps to confirm or rule out the
   suspected causes.

Rules:
- Never invent evidence that is not in the provided records.
- If the evidence is insufficient, say exactly what is still missing.
- Keep it engineering-focused and concise. Reply in the same language as the
  operator's symptom; if the language is unclear, reply in English.
"""


def explain_with_llm(client: Any, model: str, state: DiagnosticState, lang: str = "en") -> str:
    payload = {
        "symptom": state.symptom,
        "language": lang,
        "tool_calls": [
            {"tool": tc.tool, "args": tc.args, "result": tc.result[:_EXPLAINER_EVIDENCE_CHARS]}
            for tc in state.tool_calls
        ],
        "test_results": [tr.as_dict() for tr in state.test_results],
        "graph_summary": state.graph_summary,
    }
    user = (
        "Diagnose the following. The symptom and the collected evidence are "
        "provided as JSON below.\n\n"
        f"Reply in the same language as the operator's symptom (detected: "
        f"{LANGUAGE_NAMES.get(lang, 'English')}). If the language is unclear, "
        "reply in English.\n\n"
        + json.dumps(payload, ensure_ascii=False, default=str)
    )
    with console.status("[bold #7C4DFF]Writing diagnosis…", spinner="dots"):
        resp = client.chat.completions.create(
            model=model,
            messages=[
                {"role": "system", "content": EXPLAINER_SYSTEM_PROMPT},
                {"role": "user", "content": user},
            ],
            temperature=0.3,
        )
    return resp.choices[0].message.content or ""


def _print_jev_decision(label: str, decision: JevDecision) -> None:
    top = sorted(decision.probabilities.items(), key=lambda kv: kv[1], reverse=True)[:3]
    bits = " · ".join(f"{k} {v:.0%}" for k, v in top)
    console.print(
        Text(f"  🧠 Jev {label}: ", style="bold #B388FF")
        .append(decision.action, style="bold #00E5FF")
        .append(f"   (conf {decision.confidence:.2f} · {decision.model})", style="dim")
    )
    console.print(Text(f"      {bits}", style="dim"))


def _safe_jev_decision(label: str, method: Any, *args: Any) -> JevDecision | None:
    try:
        return method(*args)
    except JevError as exc:
        console.print(f"[red]Jev {label} failed: {exc}[/red]")
        return None


_JEV_PREFERRED_TYPES = (
    "sensor_msgs/msg/JointState",
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/BatteryState",
    "diagnostic_msgs/msg/DiagnosticArray",
    "geometry_msgs/msg/Twist",
    "geometry_msgs/msg/PoseStamped",
    "nav_msgs/msg/Odometry",
)


def _jev_topic_candidates(node: HarnessNode, state: DiagnosticState) -> list[str]:
    """Deterministic pre-filter before the Jev Choice.

    Jev only sees a shortlist (≤ _JEV_MAX_TOPIC_OPTIONS) that has already been
    ranked by deterministic rules: known diagnostic names, message type, symptom
    keyword overlap, and topics already sampled in the evidence. The caller
    always appends NONE_OF_THE_ABOVE so Jev is never forced to pick a bad topic.
    """
    graph = node.inspect_graph()
    state.graph_summary = graph.get("summary")
    rows = [
        (t.get("name", ""), t.get("types", []))
        for t in graph.get("topics", [])
        if t.get("name")
    ]
    symptom_tokens = set(re.findall(r"[a-z0-9_]+", state.symptom.lower()))
    already_sampled = {
        tc.args.get("topic")
        for tc in state.tool_calls
        if tc.tool in ("sample_topic", "get_topic_snapshot") and tc.args.get("topic")
    }

    def score(name: str, types: list[str]) -> int:
        low = name.lower()
        s = 0
        if any(hint in low for hint in _JEV_TOPIC_HINTS):
            s -= 4
        if any(t in _JEV_PREFERRED_TYPES for t in types):
            s -= 3
        s -= 2 * sum(1 for tok in symptom_tokens if tok in low)
        if name in already_sampled:
            s += 2  # deprioritize re-sampling the same topic
        return s

    rows.sort(key=lambda row: (score(row[0], row[1]), row[0]))
    return [name for name, _ in rows[:_JEV_MAX_TOPIC_OPTIONS]]


def _record_test_result(state: DiagnosticState, result: str) -> None:
    """Fold a run_test result into the diagnostic state as structured data."""
    try:
        data = json.loads(result)
    except (ValueError, TypeError):
        return
    if not isinstance(data, dict) or "test_id" not in data:
        return
    fields = {
        key: data.get(key)
        for key in ("test_id", "result", "summary", "evidence", "started_at", "duration_ms")
    }
    if fields["result"] is None:
        return
    fields["evidence"] = fields["evidence"] or []
    tr = TestResult(**fields)
    state.test_results.append(tr)
    for ev in tr.evidence:
        if isinstance(ev, dict):
            try:
                state.evidence.append(Evidence(**ev))
            except TypeError:
                pass


def run_jev_diagnosis(
    runtime: AgentRuntime,
    router: JevRouter,
    llm_client: Any,
    model: str,
    symptom: str,
    lang: str = "en",
) -> None:
    """System 1 routing loop with deterministic gates.

    Jev proposes the next action (and whether to finalize), but deterministic
    code decides what is allowed:

    - finalize is withheld until ≥ _JEV_MIN_EVIDENCE_ITEMS successful tool calls;
    - an exact (tool, canonical args) repeat is excluded so Jev picks another tool;
    - each tool has a per-tool call budget; an exhausted tool is excluded, not fatal;
    - the whole loop is bounded by _JEV_MAX_ROUNDS.

    All state lives in a DiagnosticState, so the loop itself does not care which
    router produced each decision.
    """
    state = DiagnosticState(symptom=symptom)
    seen: set[tuple[str, str]] = set()
    tool_counts: dict[str, int] = {}
    exhausted: set[str] = set()  # tools whose budget is spent or whose exact call repeated
    topic_candidates: list[str] | None = None
    finalized = False

    for _round in range(_JEV_MAX_ROUNDS):
        state.step = _round + 1
        meaningful_calls = sum(1 for tc in state.tool_calls if tc.ok)
        allow_finalize = meaningful_calls >= _JEV_MIN_EVIDENCE_ITEMS
        if not allow_finalize:
            console.print(
                Text(
                    f"  🔒 finalize gated: need ≥{_JEV_MIN_EVIDENCE_ITEMS} successful "
                    f"evidence call(s), have {meaningful_calls}",
                    style="dim",
                )
            )

        available = {a for a in JEV_ACTIONS if a != "finalize" and a not in exhausted}
        if not available:
            if allow_finalize:
                console.print("[yellow]All evidence tools are exhausted; finalizing.[/yellow]")
                finalized = True
            else:
                console.print("[yellow]All evidence tools are exhausted, but finalize is still gated; stopping.[/yellow]")
            break

        decision = _safe_jev_decision("next-action", router.next_action, state, allow_finalize, exhausted)
        if decision is None:
            break
        action = decision.action
        _print_jev_decision("next action", decision)

        if action == "finalize":
            # Defensive: the gate already removed finalize when evidence is
            # insufficient, so this should not happen; ignore if it does.
            if not allow_finalize:
                console.print("[yellow]Jev proposed finalize while gated; ignoring.[/yellow]")
                continue
            finalized = True
            break
        if action not in JEV_ACTIONS:
            # Whitelist guard: Jev can never reach a tool outside JEV_ACTIONS.
            console.print(f"[yellow]Jev suggested {action!r}, which is not a routable tool; skipping.[/yellow]")
            continue
        if action in exhausted:
            # Defensive: exhausted tools are removed from Jev's criteria, so this
            # should not happen; exclude it again and ask Jev to choose another.
            console.print(f"[yellow]Jev suggested exhausted tool {action!r}; excluding it and retrying.[/yellow]")
            exhausted.add(action)
            continue

        args: dict[str, Any] = {}
        if action == "run_test":
            sub = _safe_jev_decision("test", router.choose_test_id, state)
            if sub is None:
                break
            _print_jev_decision("test", sub)
            if sub.action not in TEST_CATALOG or TEST_CATALOG[sub.action].get("writes"):
                console.print(f"[yellow]Jev suggested test {sub.action!r}, which is not read-only; skipping.[/yellow]")
                continue
            args = {"test_id": sub.action}
        elif action in ("sample_topic", "get_topic_snapshot"):
            if topic_candidates is None:
                topic_candidates = _jev_topic_candidates(runtime.node, state)
            if not topic_candidates:
                console.print("[yellow]No topics discovered to sample; recording as unavailable evidence.[/yellow]")
                result = compact({"ok": False, "error": "no topics discovered"})
                state.tool_calls.append(ToolCall(tool=action, args={}, result=result, ok=False))
                tool_counts[action] = tool_counts.get(action, 0) + 1
                continue
            sub = _safe_jev_decision(
                "topic",
                router.choose_topic,
                state,
                topic_candidates,
                {_JEV_NONE_TOPIC: "None of these topics is the right one to sample next."},
            )
            if sub is None:
                break
            _print_jev_decision("topic", sub)
            if sub.action == _JEV_NONE_TOPIC:
                console.print("[yellow]Jev declined all candidate topics; recording as unavailable evidence.[/yellow]")
                result = compact({"ok": False, "error": "no relevant topic among candidates"})
                state.tool_calls.append(ToolCall(tool=action, args={}, result=result, ok=False))
                tool_counts[action] = tool_counts.get(action, 0) + 1
                continue
            args = {"topic": sub.action}

        key = (action, json.dumps(args, sort_keys=True, default=str))
        if key in seen:
            console.print(f"[yellow]Jev repeated the exact call {action} {args}; excluding {action} so Jev picks another tool.[/yellow]")
            exhausted.add(action)
            continue
        if tool_counts.get(action, 0) >= _JEV_MAX_CALLS_PER_TOOL:
            console.print(f"[yellow]Tool {action} reached its per-tool budget ({_JEV_MAX_CALLS_PER_TOOL}); excluding it so Jev picks another tool.[/yellow]")
            exhausted.add(action)
            continue
        seen.add(key)
        tool_counts[action] = tool_counts.get(action, 0) + 1

        with console.status(f"[bold #00E5FF]⚡ {action} {args}", spinner="bouncingBall"):
            result = runtime.execute(action, args)
        preview = result if len(result) <= 420 else result[:420] + " …"
        console.print(Text(f"  ⚡ {action} → {preview}", style="dim"))
        ok = _result_ok(result)
        state.tool_calls.append(ToolCall(tool=action, args=args, result=result, ok=ok))
        if action == "run_test":
            _record_test_result(state, result)

    if not state.tool_calls and not finalized:
        console.print("[yellow]No evidence was collected; nothing to explain.[/yellow]")
        return

    console.print(
        Panel(
            Text("Jev: enough evidence — handing off to the LLM explainer", style="bold #B388FF"),
            border_style="#B388FF",
            title="System 1 → System 2",
        )
    )
    explanation = explain_with_llm(llm_client, model, state, lang)
    console.print(
        Panel(
            Markdown(explanation or "(empty)"),
            border_style="#7C4DFF",
            title=f"[bold]🩺 RoboDiag ROS 2 v{VERSION}[/bold]",
        )
    )


# ──────────────────────────────────────────────────────────────────────────────
# QUERY / ACTION / CHAT handlers
#
# QUERY: resolve capability → one minimal deterministic tool call → concise
# text. These paths deliberately never call run_jev_diagnosis() or the LLM
# explainer, so a simple question stays cheap and fast.
# ──────────────────────────────────────────────────────────────────────────────
_TOPIC_RE = re.compile(r"/(?:[A-Za-z0-9_]+/)*[A-Za-z0-9_]+")


def _extract_topic(text: str, default: str | None = None) -> str | None:
    match = _TOPIC_RE.search(text)
    return match.group(0) if match else default


_LANG: dict[str, dict[str, str]] = {
    "en": {
        "battery_full": "Battery: {pct} ({v} V)",
        "battery_pct": "Battery: {pct}",
        "battery_v": "Battery voltage: {v} V",
        "battery_unavailable": "Battery: unavailable (no BatteryState received)",
        "diag_no_data": "No /diagnostics data received.",
        "diag_errors": "{n} ERROR/STALE status(es):",
        "diag_warns": "{n} WARN status(es):",
        "diag_no_errors": "No ERROR statuses reported.",
        "graph_summary": "ROS graph: {n} nodes · {t} topics · {s} services",
        "nodes_header": "{n} node(s):",
        "no_nodes": "No nodes discovered.",
        "topics_header": "{n} topic(s):",
        "no_topics": "No topics discovered.",
        "services_header": "{n} service(s):",
        "no_services": "No services discovered.",
        "more": "… and {n} more",
        "controllers_header": "Controllers:",
        "no_controllers": "No controllers reported.",
        "ros2_control_unavailable": "ros2_control unavailable",
        "joints_header": "Latest /joint_states ({n} joints):",
        "joints_no_data": "No /joint_states data received.",
        "joints_no_named": "JointState received but no named joints found.",
        "joint_pos": "{name}: {pos} rad",
        "joint_pos_unavailable": "{name}: position unavailable",
        "js_rate": "/joint_states: {rate} Hz\nSamples: {n} over {d} s",
        "js_rate_fail": "/joint_states sampling failed",
        "history_latest": "Latest {test_id}: {result}",
        "history_run": "Run: {time}",
        "history_summary": "Summary: {summary}",
        "no_history": "No test history yet.",
        "safety_allow": "Safety Gate: ALLOW motion (diagnostics, battery and joint states are fresh).",
        "safety_deny": "Safety Gate: DENY motion",
        "prompt_which_topic": "Which topic? Include a topic such as /joint_states in the question.",
        "chat_fallback": "I can answer robot questions and run diagnostics. Try asking about the battery, topics, controllers, or a symptom you're seeing.",
    },
    "zh": {
        "battery_full": "电量：{pct}（{v} V）",
        "battery_pct": "电量：{pct}",
        "battery_v": "电池电压：{v} V",
        "battery_unavailable": "电量：不可用（未收到 BatteryState）",
        "diag_no_data": "未收到 /diagnostics 数据。",
        "diag_errors": "{n} 个 ERROR/STALE 状态：",
        "diag_warns": "{n} 个 WARN 状态：",
        "diag_no_errors": "未报告 ERROR 状态。",
        "graph_summary": "ROS 图：{n} 个节点 · {t} 个话题 · {s} 个服务",
        "nodes_header": "{n} 个节点：",
        "no_nodes": "未发现节点。",
        "topics_header": "{n} 个话题：",
        "no_topics": "未发现话题。",
        "services_header": "{n} 个服务：",
        "no_services": "未发现服务。",
        "more": "… 还有 {n} 个",
        "controllers_header": "控制器：",
        "no_controllers": "未报告控制器。",
        "ros2_control_unavailable": "ros2_control 不可用",
        "joints_header": "最新 /joint_states（{n} 个关节）：",
        "joints_no_data": "未收到 /joint_states 数据。",
        "joints_no_named": "已收到 JointState，但未发现具名关节。",
        "joint_pos": "{name}：{pos} rad",
        "joint_pos_unavailable": "{name}：位置不可用",
        "js_rate": "/joint_states：{rate} Hz\n样本：{n}，{d} 秒",
        "js_rate_fail": "/joint_states 采样失败",
        "history_latest": "最新 {test_id}：{result}",
        "history_run": "运行时间：{time}",
        "history_summary": "摘要：{summary}",
        "no_history": "暂无测试历史。",
        "safety_allow": "安全门：允许运动（诊断、电量与关节状态均新鲜）。",
        "safety_deny": "安全门：禁止运动",
        "prompt_which_topic": "请指定话题，例如 /joint_states。",
        "chat_fallback": "我可以回答机器人相关问题并运行诊断。你可以询问电量、话题、控制器，或描述你看到的问题。",
    },
}


def _t(lang: str, key: str, **kwargs: Any) -> str:
    table = _LANG.get(lang) or _LANG["en"]
    template = table.get(key) or _LANG["en"].get(key) or key
    try:
        return template.format(**kwargs)
    except (KeyError, IndexError, ValueError):
        return template


def _fmt_battery(state: dict[str, Any], lang: str = "en") -> str:
    pct = state.get("percentage")
    voltage = state.get("voltage")
    if pct is not None and voltage is not None:
        return _t(lang, "battery_full", pct=f"{pct:.0%}", v=f"{voltage:.1f}")
    if pct is not None:
        return _t(lang, "battery_pct", pct=f"{pct:.0%}")
    if voltage is not None:
        return _t(lang, "battery_v", v=f"{voltage:.1f}")
    return _t(lang, "battery_unavailable")


def _fmt_diagnostics(d: dict[str, Any], lang: str = "en") -> str:
    if not d.get("available"):
        return _t(lang, "diag_no_data")
    errors = [x for x in d.get("statuses", []) if x["level"] >= DIAG_ERROR]
    warns = [x for x in d.get("statuses", []) if x["level"] == DIAG_WARN]
    lines: list[str] = []
    if errors:
        lines.append(_t(lang, "diag_errors", n=len(errors)))
        for x in errors[:5]:
            lines.append(f"- {x['name']}: {x.get('message') or ''}")
        if warns:
            lines.append(_t(lang, "diag_warns", n=len(warns)))
            for x in warns[:5]:
                lines.append(f"- {x['name']}: {x.get('message') or ''}")
    elif warns:
        lines.append(_t(lang, "diag_no_errors"))
        lines.append(_t(lang, "diag_warns", n=len(warns)))
        for x in warns[:5]:
            lines.append(f"- {x['name']}: {x.get('message') or ''}")
    else:
        lines.append(_t(lang, "diag_no_errors"))
    return "\n".join(lines)


def _fmt_graph(g: dict[str, Any], lang: str = "en") -> str:
    s = g.get("summary", {})
    return _t(
        lang,
        "graph_summary",
        n=s.get("nodes", 0),
        t=s.get("topics", 0),
        s=s.get("services", 0),
    )


def _fmt_nodes(g: dict[str, Any], lang: str = "en") -> str:
    names = [x["name"] for x in g.get("nodes", [])]
    if not names:
        return _t(lang, "no_nodes")
    out = _t(lang, "nodes_header", n=len(names))
    for n in names[:25]:
        out += f"\n- {n}"
    if len(names) > 25:
        out += f"\n- {_t(lang, 'more', n=len(names) - 25)}"
    return out


def _fmt_topics(g: dict[str, Any], lang: str = "en") -> str:
    rows = g.get("topics", [])
    if not rows:
        return _t(lang, "no_topics")
    out = _t(lang, "topics_header", n=len(rows))
    for t in rows[:25]:
        out += f"\n- {t['name']} [{', '.join(t.get('types', []))}]"
    if len(rows) > 25:
        out += f"\n- {_t(lang, 'more', n=len(rows) - 25)}"
    return out


def _fmt_services(g: dict[str, Any], lang: str = "en") -> str:
    rows = g.get("services", [])
    if not rows:
        return _t(lang, "no_services")
    out = _t(lang, "services_header", n=len(rows))
    for t in rows[:25]:
        out += f"\n- {t['name']} [{', '.join(t.get('types', []))}]"
    if len(rows) > 25:
        out += f"\n- {_t(lang, 'more', n=len(rows) - 25)}"
    return out


def _fmt_controllers(c: dict[str, Any], lang: str = "en") -> str:
    if not c.get("available"):
        return c.get("error") or _t(lang, "ros2_control_unavailable")
    controllers = (((c.get("controllers") or {}).get("data") or {}).get("controller") or [])
    if not controllers:
        return _t(lang, "no_controllers")
    out = _t(lang, "controllers_header")
    for x in controllers[:25]:
        out += f"\n- {x.get('name')} [{x.get('state', 'unknown')}]"
    return out


def _fmt_joint_states(js: dict[str, Any], lang: str = "en") -> str:
    if not js.get("available"):
        return _t(lang, "joints_no_data")
    msg = js.get("message") or {}
    joints = msg.get("_joints") or {}
    if not joints:
        return _t(lang, "joints_no_named")
    out = _t(lang, "joints_header", n=len(joints))
    for name, v in joints.items():
        pos = v.get("position")
        if pos is not None:
            out += f"\n- {_t(lang, 'joint_pos', name=name, pos=f'{pos:.3f}')}"
        else:
            out += f"\n- {_t(lang, 'joint_pos_unavailable', name=name)}"
    return out


def _fmt_joint_state_rate(s: dict[str, Any], lang: str = "en") -> str:
    if not s.get("ok"):
        return s.get("error") or _t(lang, "js_rate_fail")
    return _t(
        lang,
        "js_rate",
        rate=f"{s.get('effective_rate_hz', 0):.1f}",
        n=s.get("samples", 0),
        d=f"{s.get('duration_s', 0):.1f}",
    )


def _fmt_history(rows: list[dict[str, Any]], lang: str = "en") -> str:
    if not rows:
        return _t(lang, "no_history")
    latest = rows[0]
    lines = [_t(lang, "history_latest", test_id=latest["test_id"], result=latest["result"])]
    lines.append(_t(lang, "history_run", time=latest["started_at"]))
    lines.append(_t(lang, "history_summary", summary=latest["summary"]))
    return "\n".join(lines)


def _fmt_safety(s: dict[str, Any], lang: str = "en") -> str:
    if s.get("allow"):
        return _t(lang, "safety_allow")
    lines = [_t(lang, "safety_deny")]
    for c in [c for c in s.get("checks", []) if not c.get("ok")][:8]:
        lines.append(f"- {c.get('check')}: {c.get('reason', 'failed')}")
    return "\n".join(lines)


def handle_query(
    route: RequestRoute,
    text: str,
    node: HarnessNode,
    history_store: HistoryStore,
    gate: SafetyGate,
    lang: str = "en",
) -> bool:
    """Handle a QUERY route with one minimal deterministic tool call.

    Returns True when handled; False when the caller should fall back (for
    example, an unknown capability string).
    """
    capability = route.capability
    if capability not in QUERY_CAPABILITIES:
        return False
    spec = QUERY_CAPABILITIES[capability]
    tool = spec.tool

    if tool == "battery_state":
        console.print(_fmt_battery(node.battery_state(), lang))
    elif tool == "get_diagnostics":
        console.print(_fmt_diagnostics(node.get_diagnostics(include_ok=False), lang))
    elif tool == "inspect_graph":
        g = node.inspect_graph()
        if capability == "nodes":
            console.print(_fmt_nodes(g, lang))
        elif capability == "topics":
            console.print(_fmt_topics(g, lang))
        elif capability == "services":
            console.print(_fmt_services(g, lang))
        else:
            console.print(_fmt_graph(g, lang))
    elif tool == "inspect_ros2_control":
        console.print(_fmt_controllers(node.inspect_ros2_control(), lang))
    elif tool == "joint_state_cached":
        console.print(_fmt_joint_states(node.joint_state_cached(), lang))
    elif tool == "sample_topic":
        if capability == "joint_state_rate":
            with console.status("[bold #00E5FF]Sampling /joint_states…[/bold #00E5FF]"):
                s = node.sample_topic("/joint_states", duration_s=2.0, rate_hz=50.0)
            console.print(_fmt_joint_state_rate(s, lang))
        else:  # topic_statistics
            topic = _extract_topic(text)
            if not topic:
                console.print(f"[yellow]{_t(lang, 'prompt_which_topic')}[/yellow]")
                return True
            with console.status(f"[bold #00E5FF]Sampling {topic}…[/bold #00E5FF]"):
                s = node.sample_topic(topic)
            print_sample(s)
    elif tool == "topic_snapshot":
        topic = _extract_topic(text)
        if not topic:
            console.print(f"[yellow]{_t(lang, 'prompt_which_topic')}[/yellow]")
            return True
        with console.status(f"[bold #00E5FF]Waiting for {topic}…[/bold #00E5FF]"):
            result = node.topic_snapshot(topic)
        console.print_json(json.dumps(result, ensure_ascii=False, default=str))
    elif tool == "query_history":
        console.print(_fmt_history(history_store.recent(10), lang))
    elif tool == "check_motion_safety":
        console.print(_fmt_safety(gate.check_for_motion(), lang))
    else:
        return False
    return True


def handle_action(
    route: RequestRoute,
    node: HarnessNode,
    runner: TestRunner,
) -> bool:
    """Execute an ACTION route under deterministic policy.

    Only read-only tests and the software emergency stop are supported today.
    The AI only names the action; deterministic code validates it against the
    registry and the read-only test catalog before anything runs.
    """
    action = route.capability or ""
    if action.startswith("run_test:"):
        test_id = action.split(":", 1)[1]
        if test_id not in TEST_CATALOG or TEST_CATALOG[test_id].get("writes"):
            console.print(f"[yellow]Action {action!r} is not a permitted read-only test.[/yellow]")
            return True
        with console.status(f"[bold #00E5FF]Running {test_id}…[/bold #00E5FF]"):
            result = runner.run(test_id)
        if result.get("error"):
            console.print(f"[red]{result['error']}[/red]")
        else:
            print_test(result)
        return True
    if action == "emergency_stop":
        result = node.emergency_stop()
        console.print_json(json.dumps(result, ensure_ascii=False, default=str))
        return True
    return False


def handle_chat(llm_client: Any | None, model: str, text: str, lang: str = "en") -> None:
    """Lightweight CHAT: no tools, no telemetry. Uses the LLM only when available."""
    if llm_client is None:
        console.print(_t(lang, "chat_fallback"))
        return
    language_hint = LANGUAGE_NAMES.get(lang)
    system = (
        "You are a concise robot assistant. Reply in the same language as the "
        "user; if the user's language is unclear, reply in English. "
        "Keep answers short and do not invent telemetry."
    )
    if language_hint and lang != "en":
        system += f" The user's message appears to be in {language_hint}."
    with console.status("[bold #7C4DFF]Thinking…[/bold #7C4DFF]"):
        try:
            resp = llm_client.chat.completions.create(
                model=model,
                messages=[
                    {"role": "system", "content": system},
                    {"role": "user", "content": text},
                ],
                temperature=0.4,
            )
            content = resp.choices[0].message.content or ""
        except Exception as exc:  # noqa: BLE001
            console.print(f"[red]Chat failed: {type(exc).__name__}: {exc}[/red]")
            return
    console.print(content)


# ──────────────────────────────────────────────────────────────────────────────
# CLI rendering
# ──────────────────────────────────────────────────────────────────────────────
HELP = f"""[bold]RoboDiag ROS 2 Harness v{VERSION}[/bold]

  /graph                         ROS graph summary
  /diagnostics                   Standard /diagnostics status
  /control                       ros2_control status
  /check                         Composite read-only health check
  /topic <topic>                 Snapshot one message from a topic
  /sample <topic> \[sec] \[Hz]   Sample a topic over time and compute statistics
  /tests                         Test catalog
  /run <test_id>                 Run a deterministic test (all read-only in v0.2)
  /history \[n]                  Recent n test runs
  /safety                        Show the motion Safety Gate
  /stop                          Software stop: zero cmd_vel + optional Trigger service
  /jev                           Show Jev (System 1) routing status
  /help                          Help
  /quit                          Quit

Any other input is routed by Jev (System 1) first:
  QUERY     → one deterministic lookup → concise answer
  DIAGNOSIS → Jev evidence loop → LLM explanation
  ACTION    → deterministic read-only test / software stop
  CHAT      → lightweight reply (no tools)

Diagnosis requires TYPESAFE_API_KEY + DEEPSEEK_API_KEY. Simple queries and
actions work with only TYPESAFE_API_KEY. With only DEEPSEEK_API_KEY set, the
classic LLM function-calling agent is used.
"""

# Slash commands offered by the completion menu (name, description).
REPL_COMMANDS: list[tuple[str, str]] = [
    ("/help", "Show help"),
    ("/graph", "ROS graph summary"),
    ("/diagnostics", "Standard /diagnostics status"),
    ("/control", "ros2_control status"),
    ("/check", "Composite read-only health check"),
    ("/topic", "Snapshot one message from a topic"),
    ("/sample", "Sample a topic over a time window"),
    ("/tests", "Test catalog"),
    ("/run", "Run a deterministic test"),
    ("/history", "Recent test runs"),
    ("/safety", "Show the motion Safety Gate"),
    ("/stop", "Software stop (zero cmd_vel + optional Trigger)"),
    ("/jev", "Show Jev (System 1) routing status"),
    ("/quit", "Quit"),
]

if _HAVE_PROMPT_TOOLKIT:

    class SlashCommandCompleter(Completer):
        """Live dropdown for slash commands, topic names and test ids."""

        def __init__(
            self,
            topic_provider: Callable[[], list[str]],
            test_provider: Callable[[], dict[str, Any]],
        ) -> None:
            self._topic_provider = topic_provider
            self._test_provider = test_provider

        def get_completions(self, document: Any, complete_event: Any):
            stripped = document.text_before_cursor.lstrip()
            if not stripped.startswith("/"):
                return

            trailing_space = stripped.endswith(" ")
            tokens = stripped.split()
            if not tokens:
                return
            command = tokens[0]

            # Completing the command name itself (e.g. typing "/" or "/gr").
            if len(tokens) == 1 and not trailing_space:
                fragment = command
                for name, description in REPL_COMMANDS:
                    if name.startswith(fragment):
                        yield Completion(
                            name,
                            start_position=-len(fragment),
                            display=name,
                            display_meta=description,
                        )
                return

            # Only the first argument is completed; anything past it is freeform.
            if (trailing_space and len(tokens) >= 2) or len(tokens) >= 3:
                return
            fragment = tokens[1] if len(tokens) >= 2 else ""

            if command in ("/topic", "/sample"):
                for topic in self._topic_provider():
                    if topic.startswith(fragment):
                        yield Completion(
                            topic,
                            start_position=-len(fragment),
                            display=topic,
                            display_meta="topic",
                        )
            elif command == "/run":
                for test_id, spec in self._test_provider().items():
                    if test_id.startswith(fragment):
                        yield Completion(
                            test_id,
                            start_position=-len(fragment),
                            display=test_id,
                            display_meta=str(spec.get("name", "")),
                        )


def _make_prompt_session(node: HarnessNode) -> Any | None:
    """Create the interactive prompt with completion, or None for the fallback."""
    if not _HAVE_PROMPT_TOOLKIT or not sys.stdin.isatty():
        return None

    topic_cache: dict[str, Any] = {"at": 0.0, "items": []}

    def topic_provider() -> list[str]:
        now = time.monotonic()
        if now - topic_cache["at"] > 2.0:
            try:
                topic_cache["items"] = sorted(name for name, _ in node.get_topic_names_and_types())
            except Exception:
                pass
            topic_cache["at"] = now
        return topic_cache["items"]

    try:
        return PromptSession(
            message=_ANSI("\n\x1b[1;38;5;45mros2 ❯ \x1b[0m"),
            completer=SlashCommandCompleter(topic_provider, lambda: TEST_CATALOG),
            complete_while_typing=True,
            history=FileHistory(str(Path.home() / ".robodiag_ros2_history")),
            auto_suggest=AutoSuggestFromHistory(),
        )
    except Exception:
        return None


def print_graph(g: dict[str, Any]) -> None:
    s = g.get("summary", {})
    table = Table(title="ROS 2 Graph", border_style="#00E5FF")
    table.add_column("Metric")
    table.add_column("Value", justify="right")
    table.add_row("Nodes", str(s.get("nodes", 0)))
    table.add_row("Topics", str(s.get("topics", 0)))
    table.add_row("Services", str(s.get("services", 0)))
    console.print(table)

    important = [
        x for x in g.get("topics", [])
        if any(k in x["name"].lower() for k in ("joint", "imu", "battery", "diagnostic", "cmd_vel", "camera", "scan"))
    ][:30]
    if important:
        t = Table(title="Relevant Topics", border_style="#7C4DFF")
        t.add_column("Topic")
        t.add_column("Type(s)")
        for row in important:
            t.add_row(row["name"], ", ".join(row["types"]))
        console.print(t)


def print_diagnostics(d: dict[str, Any]) -> None:
    if not d.get("available"):
        console.print("[yellow]No standard /diagnostics data received yet.[/yellow]")
        return
    t = Table(title=f"/diagnostics · overall={d.get('overall')}", border_style="#00E5FF")
    for c in ("Level", "Name", "Message", "Age"):
        t.add_column(c, overflow="fold")
    colors = {"OK": "green", "WARN": "yellow", "ERROR": "red", "STALE": "red"}
    for row in d.get("statuses", []):
        level = row["level_name"]
        t.add_row(
            f"[{colors.get(level, 'white')}]{level}[/]",
            row["name"],
            row.get("message", ""),
            f"{row.get('age_s', '?')}s",
        )
    console.print(t)


def print_test(result: dict[str, Any]) -> None:
    status = result.get("result", "?")
    color = {"PASS": "green", "WARN": "yellow", "FAIL": "red", "SKIP": "cyan"}.get(status, "white")
    console.print(
        Panel(
            f"[{color}][bold]{status}[/bold][/{color}]  {result.get('summary', '')}\n"
            f"[dim]{result.get('test_id', '')} · {result.get('duration_ms', '?')} ms[/dim]",
            border_style=color,
            title="Diagnostic Test",
        )
    )


def print_control(c: dict[str, Any]) -> None:
    if not c.get("available"):
        console.print(f"[yellow]{c.get('error', 'ros2_control unavailable')}[/yellow]")
        return
    console.print_json(json.dumps(c, ensure_ascii=False, default=str))


def print_sample(s: dict[str, Any], max_rows: int = 30) -> None:
    if not s.get("ok"):
        console.print(f"[red]{s.get('error')}[/red]")
        return
    t = Table(
        title=f"{s['topic']} · {s['samples']} samples · {s['effective_rate_hz']} Hz",
        border_style="#00E5FF",
    )
    for c in ("Metric", "min", "max", "mean", "std", "range"):
        t.add_column(c, overflow="fold")
    stats = s.get("statistics", {})
    # Prefer derived/named fields over verbose header timestamps.
    ordered = sorted(stats.items(), key=lambda kv: (0 if "_joints" in kv[0] or "_orientation" in kv[0] else 1, kv[0]))
    for path, v in ordered[:max_rows]:
        t.add_row(path, str(v["min"]), str(v["max"]), str(v["mean"]), str(v["std"]), str(v["range"]))
    console.print(t)
    if len(stats) > max_rows:
        console.print(f"[dim]Showing only the first {max_rows}/{len(stats)} numeric fields.[/dim]")


def confirm_stop() -> bool:
    # /stop is intentionally explicit in CLI, so no extra yes/no confirmation is needed.
    return True


# ──────────────────────────────────────────────────────────────────────────────
# REPL
# ──────────────────────────────────────────────────────────────────────────────
def repl(
    node: HarnessNode,
    runner: TestRunner,
    history_store: HistoryStore,
    gate: SafetyGate,
    runtime: AgentRuntime,
    llm_client: Any | None,
    model: str,
    jev_router: JevRouter | None,
    request_router: RequestRouter | None,
) -> None:
    chat_history: list[dict[str, Any]] = [{"role": "system", "content": SYSTEM_PROMPT}]
    console.print(HELP)

    prompt_session = _make_prompt_session(node)

    while True:
        try:
            if prompt_session is not None:
                line = prompt_session.prompt().strip()
            else:
                line = console.input("\n[bold #00E5FF]ros2 ❯ [/bold #00E5FF]").strip()
        except (EOFError, KeyboardInterrupt):
            console.print("\n[dim]Goodbye 🤖[/dim]")
            break
        if not line:
            continue

        try:
            if line in ("/quit", "/q", "/exit"):
                console.print("[dim]Goodbye 🤖[/dim]")
                break
            if line == "/help":
                console.print(HELP)
                continue
            if line == "/graph":
                print_graph(node.inspect_graph())
                continue
            if line == "/diagnostics":
                print_diagnostics(node.get_diagnostics(include_ok=True))
                continue
            if line == "/control":
                with console.status("[bold #00E5FF]Reading controller_manager…[/bold #00E5FF]"):
                    c = node.inspect_ros2_control()
                print_control(c)
                continue
            if line == "/check":
                with console.status("[bold #00E5FF]Running composite diagnostics…[/bold #00E5FF]"):
                    result = runner.run("system_health")
                print_test(result)
                continue
            if line == "/tests":
                t = Table(title="Diagnostic Test Catalog", border_style="#00E5FF")
                t.add_column("ID")
                t.add_column("Name")
                t.add_column("Writes")
                t.add_column("Description")
                for tid, spec in TEST_CATALOG.items():
                    t.add_row(tid, spec["name"], "yes" if spec["writes"] else "no", spec["description"])
                console.print(t)
                continue
            if line.startswith("/run "):
                test_id = line.split(maxsplit=1)[1].strip()
                with console.status(f"[bold #00E5FF]Running {test_id}…[/bold #00E5FF]"):
                    result = runner.run(test_id)
                if result.get("error"):
                    console.print(f"[red]{result['error']}[/red]")
                else:
                    print_test(result)
                continue
            if line.startswith("/topic "):
                topic = line.split(maxsplit=1)[1].strip()
                with console.status(f"[bold #00E5FF]Waiting for {topic}…[/bold #00E5FF]"):
                    result = node.topic_snapshot(topic)
                console.print_json(json.dumps(result, ensure_ascii=False, default=str))
                continue
            if line.startswith("/sample "):
                parts = line.split()
                if len(parts) < 2:
                    console.print("[red]Usage: /sample <topic> \\[sec] \\[Hz][/red]")
                    continue
                topic = parts[1]
                try:
                    duration = float(parts[2]) if len(parts) >= 3 else 5.0
                    rate = float(parts[3]) if len(parts) >= 4 else 5.0
                except ValueError:
                    console.print("[red]Seconds and Hz must be numbers.[/red]")
                    continue
                with console.status(f"[bold #00E5FF]Sampling {topic} for {duration}s @ {rate}Hz…[/bold #00E5FF]"):
                    result = node.sample_topic(topic, duration, rate)
                print_sample(result)
                continue
            if line.startswith("/history"):
                parts = line.split()
                try:
                    n = int(parts[1]) if len(parts) > 1 else 10
                except ValueError:
                    console.print("[red]Usage: /history \\[n][/red]")
                    continue
                rows = history_store.recent(n)
                t = Table(title="Diagnostic History", border_style="#00E5FF")
                for c in ("ID", "Time", "Test", "Result", "Duration", "Summary"):
                    t.add_column(c, overflow="fold")
                for row in rows:
                    color = {"PASS": "green", "WARN": "yellow", "FAIL": "red", "SKIP": "cyan"}.get(row["result"], "white")
                    t.add_row(
                        str(row["id"]), row["started_at"], row["test_id"],
                        f"[{color}]{row['result']}[/]", f"{row['duration_ms']} ms", row["summary"][:90]
                    )
                console.print(t)
                continue
            if line == "/safety":
                console.print_json(json.dumps(gate.check_for_motion(), ensure_ascii=False, default=str))
                continue
            if line == "/stop":
                if confirm_stop():
                    result = node.emergency_stop()
                    console.print_json(json.dumps(result, ensure_ascii=False, default=str))
                continue
            if line == "/jev":
                if jev_router is not None and request_router is not None:
                    console.print(
                        f"[bold]Jev request router:[/bold] [bold #00E5FF]{request_router.model}[/bold #00E5FF] "
                        "— classifies QUERY / DIAGNOSIS / ACTION / CHAT (TypeSafe)."
                    )
                    console.print(
                        f"[bold]Jev diagnostic router:[/bold] [bold #00E5FF]{jev_router.model}[/bold #00E5FF] "
                        "— next-tool routing inside DIAGNOSIS. "
                        "It only routes read-only evidence tools; emergency_stop stays behind /stop."
                    )
                elif jev_router is not None:
                    console.print(
                        f"[bold]Jev router:[/bold] [bold #00E5FF]{jev_router.model}[/bold #00E5FF] "
                        "— System 1 next-tool routing (TypeSafe). "
                        "It only routes read-only evidence tools; emergency_stop stays behind /stop."
                    )
                else:
                    console.print("[yellow]Jev router disabled. Set TYPESAFE_API_KEY to enable System 1 routing.[/yellow]")
                continue
            if line.startswith("/"):
                console.print(f"[red]Unknown command {line}; type /help for help.[/red]")
                continue

            lang = detect_language(line)

            if request_router is not None:
                try:
                    route = request_router.route(line)
                except JevError as exc:
                    console.print(f"[red]Jev request routing failed: {exc}[/red]")
                    continue

                if route.mode is RequestMode.QUERY:
                    if handle_query(route, line, node, history_store, gate, lang=lang):
                        continue
                    console.print("[yellow]Query capability unresolved; falling back to diagnosis.[/yellow]")
                    route = RequestRoute(mode=RequestMode.DIAGNOSIS)

                if route.mode is RequestMode.ACTION:
                    if handle_action(route, node, runner):
                        continue
                    console.print("[yellow]Action unresolved; falling back to diagnosis.[/yellow]")
                    route = RequestRoute(mode=RequestMode.DIAGNOSIS)

                if route.mode is RequestMode.CHAT:
                    handle_chat(llm_client, model, line, lang=lang)
                    continue

                # DIAGNOSIS — the existing Jev evidence loop + LLM explainer.
                if jev_router is None:
                    if llm_client is None:
                        console.print("[red]AI Agent not configured. Set DEEPSEEK_API_KEY to enable natural-language diagnosis.[/red]")
                        continue
                    chat_history = agent_turn(
                        runtime,
                        llm_client,
                        model,
                        chat_history + [{"role": "user", "content": line}],
                    )
                    continue
                if llm_client is None:
                    console.print("[red]Jev router is on, but the LLM explainer is not. Set DEEPSEEK_API_KEY to enable the final diagnosis.[/red]")
                    continue
                run_jev_diagnosis(runtime, jev_router, llm_client, model, line, lang=lang)
                continue

            # No Jev request router: preserve the classic LLM function-calling agent.
            if llm_client is None:
                console.print("[red]AI Agent not configured. Set DEEPSEEK_API_KEY to enable natural-language diagnosis.[/red]")
                continue
            chat_history = agent_turn(
                runtime,
                llm_client,
                model,
                chat_history + [{"role": "user", "content": line}],
            )

        except KeyboardInterrupt:
            console.print("[yellow]Current operation interrupted; if the robot is moving abnormally, type /stop.[/yellow]")
        except Exception as exc:  # noqa: BLE001
            console.print(f"[red]{type(exc).__name__}: {exc}[/red]")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def build_llm_client(
    api_key_override: str | None = None,
    base_url_override: str | None = None,
    model_override: str | None = None,
) -> tuple[Any | None, str]:
    env_file = load_env(Path.home() / ".robodiag.env")
    # Precedence: CLI flag > environment variable > ~/.robodiag.env > built-in default.
    api_key = api_key_override or os.environ.get("DEEPSEEK_API_KEY") or env_file.get("DEEPSEEK_API_KEY", "")
    base_url = base_url_override or os.environ.get("DEEPSEEK_BASE_URL") or env_file.get("DEEPSEEK_BASE_URL", "https://api.deepseek.com")
    model = model_override or os.environ.get("DEEPSEEK_MODEL") or env_file.get("DEEPSEEK_MODEL", "deepseek-chat")
    if not api_key or "your-api-key" in api_key:
        return None, model
    if OpenAI is None:
        console.print("[yellow]API key detected, but the 'openai' Python package is missing. Run: pip install openai[/yellow]")
        return None, model
    try:
        return OpenAI(api_key=api_key, base_url=base_url), model
    except Exception as exc:  # noqa: BLE001
        console.print(f"[yellow]AI Agent initialization failed; disabled: {type(exc).__name__}: {exc}[/yellow]")
        return None, model


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="RoboDiag ROS 2 Diagnostic Harness")
    parser.add_argument("--controller-manager", default="/controller_manager", help="controller_manager node namespace")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="fallback software-stop Twist topic")
    parser.add_argument("--estop-service", default="", help="optional std_srvs/Trigger E-stop service")
    parser.add_argument("--db", default=os.environ.get("ROBODIAG_DB", "~/.robodiag_ros2.db"))
    parser.add_argument("--model", default="", help="LLM model name; overrides DEEPSEEK_MODEL")
    parser.add_argument("--base-url", default="", help="OpenAI-compatible base URL; overrides DEEPSEEK_BASE_URL")
    parser.add_argument(
        "--api-key",
        default="",
        help="LLM API key; overrides DEEPSEEK_API_KEY (WARNING: visible in `ps` and shell history)",
    )
    parser.add_argument("--api-key-file", default="", help="read the API key from a file (safer than --api-key)")
    parser.add_argument("--jev-model", default="", help="Jev model name; overrides TYPESAFE_MODEL")
    parser.add_argument("--jev-base-url", default="", help="TypeSafe base URL; overrides TYPESAFE_BASE_URL")
    parser.add_argument("--jev-api-key", default="", help="TypeSafe API key; overrides TYPESAFE_API_KEY (visible in `ps`/shell history)")
    parser.add_argument("--jev-api-key-file", default="", help="read the TypeSafe API key from a file (safer than --jev-api-key)")
    args, ros_unknown = parser.parse_known_args(argv)

    play_banner()

    # Pass ROS-specific args through to rclpy; our own args were already parsed.
    # rclpy/rcl treat args[0] as the program name and only scan args[1:] for
    # the --ros-args marker. parse_known_args strips argv[0], so re-add it or
    # ROS remappings/params would be silently dropped.
    rclpy.init(args=[sys.argv[0], *ros_unknown])
    node = HarnessNode(
        controller_manager=args.controller_manager,
        cmd_vel_topic=args.cmd_vel_topic,
        estop_service=args.estop_service or None,
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, name="robodiag-rclpy", daemon=True)
    spin_thread.start()

    # Allow DDS discovery and first common telemetry samples to arrive.
    time.sleep(0.8)
    node._ensure_common_subscriptions()

    history_store = HistoryStore(Path(args.db))
    gate = SafetyGate(
        node=node,
        min_battery_pct=env_float("ROBODIAG_MIN_BATTERY_PCT", 0.10),
        min_battery_v=env_float("ROBODIAG_MIN_BATTERY_V", 0.0),
        max_diag_age_s=env_float("ROBODIAG_MAX_DIAG_AGE_S", 5.0),
        max_joint_age_s=env_float("ROBODIAG_MAX_JOINT_STATE_AGE_S", 1.0),
        max_battery_age_s=env_float("ROBODIAG_MAX_BATTERY_AGE_S", 5.0),
    )
    runner = TestRunner(
        node,
        history_store,
        max_diag_age_s=env_float("ROBODIAG_MAX_DIAG_AGE_S", 5.0),
    )
    runtime = AgentRuntime(node, runner, history_store, gate)

    cli_api_key = args.api_key
    if not cli_api_key and args.api_key_file:
        try:
            cli_api_key = Path(args.api_key_file).expanduser().read_text(encoding="utf-8").strip()
        except OSError as exc:
            print(f"robodiag: cannot read --api-key-file: {exc}", file=sys.stderr)
            return 2
    if args.api_key:
        console.print(
            "[yellow]Warning: --api-key is visible in `ps` and shell history; "
            "prefer DEEPSEEK_API_KEY or --api-key-file.[/yellow]"
        )
    llm_client, model = build_llm_client(
        api_key_override=cli_api_key or None,
        base_url_override=args.base_url or None,
        model_override=args.model or None,
    )

    jev_api_key = args.jev_api_key
    if not jev_api_key and args.jev_api_key_file:
        try:
            jev_api_key = Path(args.jev_api_key_file).expanduser().read_text(encoding="utf-8").strip()
        except OSError as exc:
            print(f"robodiag: cannot read --jev-api-key-file: {exc}", file=sys.stderr)
            return 2
    if args.jev_api_key:
        console.print(
            "[yellow]Warning: --jev-api-key is visible in `ps` and shell history; "
            "prefer TYPESAFE_API_KEY or --jev-api-key-file.[/yellow]"
        )
    jev_router, _jev_model = build_jev_router(
        api_key_override=jev_api_key or None,
        base_url_override=args.jev_base_url or None,
        model_override=args.jev_model or None,
    )
    request_router, _request_model = build_request_router(
        api_key_override=jev_api_key or None,
        base_url_override=args.jev_base_url or None,
        model_override=args.jev_model or None,
    )

    graph = node.inspect_graph()
    console.print(
        f"  ✅ ROS graph: [bold]{graph['summary']['nodes']}[/bold] nodes · "
        f"[bold]{graph['summary']['topics']}[/bold] topics · "
        f"[bold]{graph['summary']['services']}[/bold] services"
    )
    console.print(f"  🎮 software stop topic: [bold]{args.cmd_vel_topic}[/bold]")
    if args.estop_service:
        console.print(f"  🛑 Trigger E-stop service: [bold]{args.estop_service}[/bold]")
    console.print(
        f"  🤖 Agent: [bold]{model}[/bold]"
        if llm_client
        else "  🤖 Agent: [yellow]disabled (DEEPSEEK_API_KEY not found)[/yellow]"
    )
    if request_router:
        console.print(
            f"  🧠 Jev request router: [bold #00E5FF]{_request_model}[/bold #00E5FF] (QUERY/DIAGNOSIS/ACTION/CHAT)"
        )
        console.print(
            f"  🧠 Jev diagnostic router: [bold #00E5FF]{_jev_model}[/bold #00E5FF] (next-tool routing)"
        )
    else:
        console.print(
            "  🧠 Jev router: [yellow]disabled (TYPESAFE_API_KEY not found) — falling back to LLM function calling[/yellow]"
        )
    console.print(f"  🗃  History DB: [dim]{Path(args.db).expanduser()}[/dim]\n")

    try:
        repl(node, runner, history_store, gate, runtime, llm_client, model, jev_router, request_router)
    finally:
        try:
            executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            executor.remove_node(node)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

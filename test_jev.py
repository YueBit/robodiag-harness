#!/usr/bin/env python3
"""Offline smoke test for the Jev (System One) router.

Runs without ROS 2 or `rich` — it imports robodiag_jev directly and makes a few
real calls to the TypeSafe API to verify the router works end-to-end.

Usage:
    python3 test_jev.py

API key resolution:
    1. TYPESAFE_API_KEY environment variable
    2. TYPESAFE_API_KEY in ~/.robodiag.env

Exit codes:
    0  all checks passed (or skipped because no API key was found)
    1  a check failed
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robodiag_jev import (  # noqa: E402
    JEV_ACTIONS,
    DiagnosticState,
    JevClient,
    JevError,
    JevRouter,
    ToolCall,
)

# Mirror of RoboDiag's read-only TEST_CATALOG (for choose_test_id only).
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

SYMPTOM = "My robot shakes badly when it walks."


def load_api_key() -> str:
    key = os.environ.get("TYPESAFE_API_KEY", "").strip()
    if key:
        return key
    env_file = Path.home() / ".robodiag.env"
    if env_file.exists():
        for line in env_file.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line.startswith("TYPESAFE_API_KEY="):
                return line.split("=", 1)[1].strip()
    return ""


def main() -> int:
    api_key = load_api_key()
    if not api_key:
        print("SKIP: no TYPESAFE_API_KEY found (set it or add it to ~/.robodiag.env)")
        return 0

    client = JevClient(api_key=api_key)
    router = JevRouter(client, TEST_CATALOG)
    failures = 0

    def check(label: str, ok: bool, detail: str = "") -> None:
        nonlocal failures
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {label}" + (f"  → {detail}" if detail else ""))
        if not ok:
            failures += 1

    # 1. Next action on a fresh state (no evidence yet).
    state = DiagnosticState(symptom=SYMPTOM)
    d = router.next_action(state)
    check(
        "next_action returns a routable action",
        d.action in JEV_ACTIONS,
        f"{d.action} (conf {d.confidence:.2f})",
    )

    # 2. Next action after a diagnostics warning was collected.
    evidence_state = DiagnosticState(
        symptom=SYMPTOM,
        tool_calls=[
            ToolCall(
                tool="get_diagnostics",
                args={},
                result='{"ok": true, "overall": "WARN", "statuses": [{"name": "motor_controller", "level_name": "WARN", "message": "joint_temperature high"}]}',
                ok=True,
            )
        ],
    )
    d2 = router.next_action(evidence_state)
    check(
        "next_action adapts to collected evidence",
        d2.action in JEV_ACTIONS,
        f"{d2.action} (conf {d2.confidence:.2f})",
    )

    # 3. Test selection is constrained to the injected catalog.
    t = router.choose_test_id(evidence_state)
    check("choose_test_id stays inside the read-only catalog", t.action in TEST_CATALOG, t.action)

    # 4. Topic selection is constrained to the supplied topics.
    topics = ["/joint_states", "/imu/data", "/cmd_vel", "/battery_state"]
    top = router.choose_topic(evidence_state, topics)
    check("choose_topic stays inside the supplied topics", top.action in topics, top.action)

    # 5. Finalize gate: with allow_finalize=False, Jev must not choose finalize.
    d3 = router.next_action(state, allow_finalize=False)
    check(
        "next_action honors the finalize gate (finalize withheld)",
        d3.action != "finalize" and d3.action in JEV_ACTIONS,
        d3.action,
    )

    # 6. NONE_OF_THE_ABOVE is respected when supplied as an extra option.
    none_key = "none_of_the_above"
    top2 = router.choose_topic(
        evidence_state,
        topics,
        {none_key: "None of these topics is the right one to sample next."},
    )
    check(
        "choose_topic honors the NONE_OF_THE_ABOVE option",
        top2.action in topics + [none_key],
        top2.action,
    )

    # 7. Probabilities are sane (sum ~1).
    probs_ok = 0.95 <= sum(d.probabilities.values()) <= 1.05
    check("choice probabilities sum to ~1", probs_ok, f"{sum(d.probabilities.values()):.3f}")

    print()
    if failures:
        print(f"{failures} check(s) FAILED")
        return 1
    print("All Jev smoke checks passed ✔")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except JevError as exc:
        print(f"JevError: {exc}", file=sys.stderr)
        raise SystemExit(1)

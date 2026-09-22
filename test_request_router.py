#!/usr/bin/env python3
"""Offline + optional live smoke tests for the Jev RequestRouter.

Two layers:

1. Deterministic wiring tests (always run, no network): a scripted fake
   JevClient is used to prove that RequestRouter.route() maps Jev choices to
   the correct RequestRoute and that unknown capability labels are rejected.

2. Live classification tests (run only when TYPESAFE_API_KEY is available):
   real Jev calls verify the QUERY / DIAGNOSIS / ACTION cases from the spec.

Usage:
    python3 test_request_router.py

Exit codes:
    0  all checks passed (or skipped)
    1  a check failed
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from robodiag_jev import (  # noqa: E402
    LANGUAGE_NAMES,
    QUERY_CAPABILITIES,
    JevClient,
    QueryCapability,
    RequestMode,
    RequestRouter,
    detect_language,
)

# Read-only action registry mirroring RoboDiag's TEST_CATALOG + emergency_stop.
ACTION_CAPABILITIES: dict[str, QueryCapability] = {
    "emergency_stop": QueryCapability(
        "emergency_stop",
        "Software stop: publish zero Twist and optionally call the configured Trigger E-stop service.",
        "emergency_stop",
    ),
    "run_test:graph_health": QueryCapability("run_test:graph_health", "Run graph_health", "run_test"),
    "run_test:diagnostics_health": QueryCapability("run_test:diagnostics_health", "Run diagnostics_health", "run_test"),
    "run_test:joint_states_health": QueryCapability("run_test:joint_states_health", "Run joint_states_health", "run_test"),
    "run_test:ros2_control_health": QueryCapability("run_test:ros2_control_health", "Run ros2_control_health", "run_test"),
    "run_test:system_health": QueryCapability("run_test:system_health", "Run system_health", "run_test"),
}

QUERY_CASES = [
    ("What is the battery level?", "battery_level", {"battery_level", "battery_voltage"}),
    ("How much charge is left?", "battery_level", {"battery_level", "battery_voltage"}),
    ("现在电量多少？", "battery_level", {"battery_level", "battery_voltage"}),
    ("What nodes are running?", "nodes", {"nodes"}),
    ("What is the joint_states rate?", "joint_state_rate", {"joint_state_rate"}),
    ("Any diagnostic errors?", "diagnostics", {"diagnostics"}),
    ("Show the last health test.", "test_history", {"test_history"}),
]

DIAGNOSIS_CASES = [
    "Why does the robot shake when walking?",
    "The left leg sometimes freezes.",
    "Why is the IMU unstable?",
    "帮我排查一下为什么机器人一直抖。",
]

ACTION_CASES = [
    ("Run system_health.", "run_test:system_health"),
    ("Run the joint state health test.", "run_test:joint_states_health"),
    ("Stop the robot.", "emergency_stop"),
]


class FakeJevClient:
    """Scripted stand-in for JevClient; returns canned choices per question key."""

    def __init__(self, choices: dict[str, str], confidence: float = 0.9):
        self.choices = choices
        self.confidence = confidence
        self.model = "fake"
        self.calls: list[tuple[dict, dict]] = []

    def system_one(self, state, questions):
        self.calls.append((state, dict(questions)))
        answers = {}
        for key, spec in questions.items():
            answers[key] = {
                "type": spec["type"],
                "choice": self.choices.get(key, ""),
                "confidence": self.confidence,
                "probabilities": {},
            }
        return {"answers": answers, "model": "fake"}


def main() -> int:
    failures = 0

    def check(label: str, ok: bool, detail: str = "") -> None:
        nonlocal failures
        status = "PASS" if ok else "FAIL"
        print(f"[{status}] {label}" + (f"  → {detail}" if detail else ""))
        if not ok:
            failures += 1

    # ── Deterministic wiring tests (offline) ─────────────────────────────────
    print("== deterministic wiring (offline) ==")

    router = RequestRouter(
        FakeJevClient({"request_mode": "query", "query_capability": "battery_level"}),
        QUERY_CAPABILITIES,
        ACTION_CAPABILITIES,
    )
    route = router.route("what is the battery level?")
    check("QUERY routes to a RequestRoute with capability", route.mode is RequestMode.QUERY, repr(route))
    check("QUERY capability is resolved", route.capability == "battery_level", repr(route))

    router = RequestRouter(
        FakeJevClient({"request_mode": "diagnosis"}),
        QUERY_CAPABILITIES,
        ACTION_CAPABILITIES,
    )
    route = router.route("the robot shakes when walking")
    check("DIAGNOSIS routes with no capability", route.mode is RequestMode.DIAGNOSIS and route.capability is None, repr(route))

    router = RequestRouter(
        FakeJevClient({"request_mode": "action", "action_capability": "run_test:system_health"}),
        QUERY_CAPABILITIES,
        ACTION_CAPABILITIES,
    )
    route = router.route("run system_health")
    check("ACTION routes to a deterministic action", route.mode is RequestMode.ACTION, repr(route))
    check("ACTION capability is resolved", route.capability == "run_test:system_health", repr(route))

    router = RequestRouter(
        FakeJevClient({"request_mode": "chat"}),
        QUERY_CAPABILITIES,
        ACTION_CAPABILITIES,
    )
    route = router.route("hello there")
    check("CHAT routes with no capability", route.mode is RequestMode.CHAT and route.capability is None, repr(route))

    # Unknown capability labels must never leak through as trusted strings.
    router = RequestRouter(
        FakeJevClient({"request_mode": "query", "query_capability": "emergency_stop"}),
        QUERY_CAPABILITIES,
        ACTION_CAPABILITIES,
    )
    route = router.route("what is the battery level?")
    check("unknown capability label is rejected", route.capability is None, repr(route))

    # Unknown mode labels fall back to DIAGNOSIS.
    router = RequestRouter(
        FakeJevClient({"request_mode": "not_a_real_mode"}),
        QUERY_CAPABILITIES,
        ACTION_CAPABILITIES,
    )
    route = router.route("anything")
    check("unknown mode label falls back to DIAGNOSIS", route.mode is RequestMode.DIAGNOSIS, repr(route))

    # Language detection: script-based, defaulting to English when ambiguous.
    print("\n== language detection (offline) ==")
    lang_cases = [
        ("What is the battery level?", "en"),
        ("现在电量多少？", "zh"),
        ("バッテリーは何％？", "ja"),
        ("배터리 얼마나 남았어?", "ko"),
        ("Какой заряд батареи?", "ru"),
        ("كم نسبة البطارية؟", "ar"),
        ("¿Cuánta batería queda?", "en"),  # Latin script is ambiguous → English default
    ]
    for text, expected in lang_cases:
        got = detect_language(text)
        check(f"detect_language({text!r})", got == expected, got)
    check("LANGUAGE_NAMES resolves zh", LANGUAGE_NAMES.get("zh") == "Chinese", str(LANGUAGE_NAMES.get("zh")))

    # ── Live classification tests (require TYPESAFE_API_KEY) ────────────────
    api_key = os.environ.get("TYPESAFE_API_KEY", "").strip()
    if not api_key:
        env_file = Path.home() / ".robodiag.env"
        if env_file.exists():
            for line in env_file.read_text(encoding="utf-8").splitlines():
                line = line.strip()
                if line.startswith("TYPESAFE_API_KEY="):
                    api_key = line.split("=", 1)[1].strip()
    if not api_key:
        print("\nSKIP: no TYPESAFE_API_KEY found; live classification checks skipped")
        print()
        return 1 if failures else 0

    print("\n== live classification (real Jev) ==")
    try:
        client = JevClient(api_key=api_key)
        router = RequestRouter(client, QUERY_CAPABILITIES, ACTION_CAPABILITIES)
    except Exception as exc:  # noqa: BLE001
        print(f"FAIL: could not build live RequestRouter: {exc}")
        return 1

    for text, expected, allowed in QUERY_CASES:
        try:
            route = router.route(text)
        except Exception as exc:  # noqa: BLE001
            check(f"QUERY '{text}'", False, f"exception: {exc}")
            continue
        ok = route.mode is RequestMode.QUERY and route.capability in allowed
        check(f"QUERY '{text}'", ok, f"mode={route.mode.value} capability={route.capability}")

    for text in DIAGNOSIS_CASES:
        try:
            route = router.route(text)
        except Exception as exc:  # noqa: BLE001
            check(f"DIAGNOSIS '{text}'", False, f"exception: {exc}")
            continue
        check(f"DIAGNOSIS '{text}'", route.mode is RequestMode.DIAGNOSIS, f"mode={route.mode.value}")

    for text, expected in ACTION_CASES:
        try:
            route = router.route(text)
        except Exception as exc:  # noqa: BLE001
            check(f"ACTION '{text}'", False, f"exception: {exc}")
            continue
        ok = route.mode is RequestMode.ACTION and route.capability == expected
        check(f"ACTION '{text}'", ok, f"mode={route.mode.value} capability={route.capability}")

    print()
    if failures:
        print(f"{failures} check(s) FAILED")
        return 1
    print("All RequestRouter checks passed ✔")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

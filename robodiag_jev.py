#!/usr/bin/env python3
"""RoboDiag Jev (System One) decision layer — no ROS, no rich.

This module is intentionally dependency-free (stdlib only) so the Jev router can
be imported and smoke-tested without a ROS 2 environment. It contains:

- the request router (RequestRouter) that classifies QUERY/DIAGNOSIS/ACTION/CHAT
- the diagnostic state model (DiagnosticState, ToolCall, TestResult, Evidence)
- the TypeSafe API client (JevClient)
- the narrow "which tool next?" diagnostic router (JevRouter)

Jev is not a text-generating LLM. It takes a state (symptom + evidence) and a
typed question, and returns a structured decision with calibrated
probabilities.

    Jev decides what to investigate.
    Deterministic tools decide what is true.
    The LLM explains what it means.
    The Safety Gate decides what is allowed.

API: POST https://api.typesafe.ai/v1/systemone  (Authorization: Bearer <key>)
See https://docs.typesafe.ai/api
"""

from __future__ import annotations

import json
import socket
import time
import urllib.error
import urllib.request
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any

TYPESAFE_DEFAULT_BASE_URL = "https://api.typesafe.ai"
TYPESAFE_DEFAULT_MODEL = "jev-latest"
TYPESAFE_ENDPOINT = "/v1/systemone"

# Jev may route to these read-only tools only. emergency_stop and any future
# motion/write tool are deliberately absent: they stay behind the deterministic
# Safety Gate and the /stop command, never behind a model decision.
JEV_ACTIONS: dict[str, str] = {
    "inspect_graph": "Inspect the ROS graph when basic connectivity or available interfaces are unclear.",
    "get_diagnostics": "Inspect standard ROS diagnostics for reported faults and warnings.",
    "inspect_ros2_control": "Inspect controllers and hardware interfaces when controller or hardware state is in question.",
    "get_topic_snapshot": "Read one message from a topic when a single point-in-time value is enough.",
    "sample_topic": "Collect time-series telemetry when jitter, instability or intermittency is suspected.",
    "list_tests": "List the deterministic health tests when unsure which test to run.",
    "run_test": "Run an existing deterministic health test when a structured check would confirm a hypothesis.",
    "query_history": "Review previous health-test results when historical comparison may help.",
    "check_motion_safety": "Check the deterministic motion Safety Gate (read-only) before any motion-related decision.",
    "finalize": "Enough evidence exists to explain the likely problem and recommend next steps.",
}

JEV_NEXT_ACTION_INSTRUCTIONS = (
    "Given the `symptom` and the `evidence` already collected, choose the most "
    "useful next diagnostic action. Prefer the most decisive missing evidence. "
    "Choose finalize only when enough evidence exists to explain the likely "
    "problem and recommend next steps."
)

# Max characters of each tool result included in the Jev `state` payload.
JEV_PREVIEW_CHARS = 700


# ──────────────────────────────────────────────────────────────────────────────
# Language detection
#
# Small, dependency-free helper so the harness can mirror the operator's
# language. Script-based detection is deliberately conservative: Latin-script
# text cannot be disambiguated by script alone, so it defaults to English.
# ──────────────────────────────────────────────────────────────────────────────
LANGUAGE_NAMES: dict[str, str] = {
    "en": "English",
    "zh": "Chinese",
    "ja": "Japanese",
    "ko": "Korean",
    "ru": "Russian",
    "ar": "Arabic",
}


def detect_language(text: str) -> str:
    """Return a best-effort ISO 639-1 language code for `text`.

    Defaults to English ("en") when the language cannot be determined from the
    script (for example Latin-script text that could be several languages).
    """
    for ch in text:
        code = ord(ch)
        if 0x4E00 <= code <= 0x9FFF:
            return "zh"
        if 0x3040 <= code <= 0x30FF:
            return "ja"
        if 0xAC00 <= code <= 0xD7AF:
            return "ko"
        if 0x0400 <= code <= 0x04FF:
            return "ru"
        if 0x0600 <= code <= 0x06FF or 0x0750 <= code <= 0x077F:
            return "ar"
    return "en"


# ──────────────────────────────────────────────────────────────────────────────
# Diagnostic state model
#
# DiagnosticState is the single object passed between the routing layer and the
# deterministic tools. Routers (Jev now; LLM / rule-based / replay later) all
# consume and produce decisions against this same shape, so the harness below
# does not need to know which router made the choice.
# ──────────────────────────────────────────────────────────────────────────────
@dataclass
class Evidence:
    source: str
    metric: str
    value: Any
    timestamp: str
    quality: str = "measured"


@dataclass
class TestResult:
    test_id: str
    result: str  # PASS / WARN / FAIL / SKIP
    summary: str
    evidence: list[dict[str, Any]]
    started_at: str
    duration_ms: int

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ToolCall:
    tool: str
    args: dict[str, Any]
    result: str  # compact JSON string returned by the deterministic tool
    ok: bool


@dataclass
class DiagnosticState:
    symptom: str
    evidence: list[Evidence] = field(default_factory=list)
    test_results: list[TestResult] = field(default_factory=list)
    tool_calls: list[ToolCall] = field(default_factory=list)
    graph_summary: dict[str, Any] | None = None
    step: int = 0


class JevError(RuntimeError):
    pass


class JevClient:
    """Minimal, dependency-free client for the TypeSafe System One endpoint.

    The official `typesafe-sdk` is a drop-in alternative; this class implements
    only the subset RoboDiag needs, using stdlib urllib so a stock ROS 2 image
    needs no extra pip install. Because every call goes through this one class,
    swapping to the SDK later is a local change.
    """

    def __init__(
        self,
        api_key: str,
        base_url: str = TYPESAFE_DEFAULT_BASE_URL,
        model: str = TYPESAFE_DEFAULT_MODEL,
        timeout: float = 30.0,
    ):
        self.api_key = api_key
        self.base_url = base_url.rstrip("/")
        self.model = model
        self.timeout = timeout

    def system_one(self, state: Any, questions: dict[str, Any]) -> dict[str, Any]:
        url = f"{self.base_url}{TYPESAFE_ENDPOINT}"
        payload = json.dumps(
            {"state": state, "model": self.model, "questions": questions},
            ensure_ascii=False,
        )
        data = payload.encode("utf-8")
        headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json",
        }

        last_error: JevError | None = None
        for attempt in range(4):
            try:
                req = urllib.request.Request(url, data=data, headers=headers, method="POST")
                with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                    return json.loads(resp.read().decode("utf-8"))
            except urllib.error.HTTPError as exc:
                detail = ""
                try:
                    detail = exc.read().decode("utf-8", "replace")[:300]
                except OSError:
                    pass
                message = f"Jev HTTP {exc.code}: {detail or exc.reason}"
                # Auth and validation errors are not transient; surface them now.
                if exc.code in (401, 422):
                    raise JevError(message) from exc
                last_error = JevError(message)
            except (urllib.error.URLError, socket.timeout, TimeoutError, OSError) as exc:
                last_error = JevError(f"Jev connection error: {type(exc).__name__}: {exc}")
            if attempt < 3:
                time.sleep(min(2 ** attempt, 8))  # 1s, 2s, 4s backoff
        raise JevError(f"Jev request failed after retries: {last_error}")


@dataclass
class JevDecision:
    action: str
    probabilities: dict[str, float]
    confidence: float
    model: str


class JevRouter:
    """System 1 router: Jev picks the next diagnostic tool.

    The router is intentionally narrow. It only answers "what should we
    investigate next?", and, when a parameterised tool is chosen, a follow-up
    question picks the concrete test_id or topic. It never selects
    emergency_stop or motion/write actions.

    It consumes a DiagnosticState so it can later be swapped for another router
    (LLM, rule-based, replay) without the harness below knowing which one made
    the decision.
    """

    def __init__(self, client: JevClient, test_catalog: dict[str, Any] | None = None):
        self.client = client
        self.test_catalog = test_catalog or {}

    @property
    def model(self) -> str:
        return self.client.model

    @staticmethod
    def _to_jev_state(state: DiagnosticState) -> dict[str, Any]:
        return {
            "symptom": state.symptom,
            "evidence": [
                {"tool": tc.tool, "args": tc.args, "preview": tc.result[:JEV_PREVIEW_CHARS]}
                for tc in state.tool_calls
            ],
        }

    def next_action(
        self,
        state: DiagnosticState,
        allow_finalize: bool = True,
        exclude: set[str] | frozenset[str] | None = None,
    ) -> JevDecision:
        criteria: dict[str, str] = dict(JEV_ACTIONS)
        instructions = JEV_NEXT_ACTION_INSTRUCTIONS
        if not allow_finalize:
            # Deterministic gate: finalize is withheld until enough evidence has
            # been collected, so it is removed from the allowed set and the
            # instruction is made explicit.
            criteria.pop("finalize", None)
            instructions = (
                "Given the `symptom` and the `evidence` already collected, choose "
                "the most useful next diagnostic action. Not enough evidence has "
                "been collected yet, so finalizing is not an option."
            )
        for name in exclude or ():
            # Budget/repeat exclusions: tools the caller has already exhausted are
            # removed from the allowed set so Jev switches to another tool instead
            # of the loop aborting.
            criteria.pop(name, None)
        if not criteria:
            raise JevError("no routable actions remain after exclusions")
        answer, model = self._ask(
            "next_action", "choice", instructions, criteria, self._to_jev_state(state)
        )
        return JevDecision(
            answer["choice"],
            answer.get("probabilities", {}),
            answer.get("confidence", 0.0),
            model,
        )

    def choose_test_id(self, state: DiagnosticState) -> JevDecision:
        criteria = {
            test_id: spec["description"]
            for test_id, spec in self.test_catalog.items()
            if not spec.get("writes")
        }
        answer, model = self._ask(
            "test_id",
            "choice",
            "Given the `symptom` and the `evidence`, which deterministic health "
            "test should run next?",
            criteria,
            self._to_jev_state(state),
        )
        return JevDecision(
            answer["choice"],
            answer.get("probabilities", {}),
            answer.get("confidence", 0.0),
            model,
        )

    def choose_topic(
        self,
        state: DiagnosticState,
        topics: list[str],
        extra_criteria: dict[str, Any] | None = None,
    ) -> JevDecision:
        criteria: dict[str, Any] = {t: None for t in topics}
        if extra_criteria:
            criteria.update(extra_criteria)
        answer, model = self._ask(
            "topic",
            "choice",
            "Given the `symptom` and the `evidence`, which ROS 2 topic should "
            "be sampled or snapshotted next?",
            criteria,
            self._to_jev_state(state),
        )
        return JevDecision(
            answer["choice"],
            answer.get("probabilities", {}),
            answer.get("confidence", 0.0),
            model,
        )

    def _ask(
        self,
        key: str,
        qtype: str,
        instructions: str,
        criteria: Any,
        state: dict[str, Any],
    ) -> tuple[dict[str, Any], str]:
        response = self.client.system_one(
            state=state,
            questions={key: {"type": qtype, "instructions": instructions, "criteria": criteria}},
        )
        answers = response.get("answers", {})
        if key not in answers:
            raise JevError(f"Jev response missing answer for {key!r}: {response}")
        answer = answers[key]
        if answer.get("type") != qtype:
            raise JevError(f"Jev returned {answer.get('type')} for a {qtype} question")
        return answer, response.get("model", self.client.model)


# ──────────────────────────────────────────────────────────────────────────────
# Request routing (System 1)
#
# Jev's first responsibility is to understand WHAT KIND of task the operator is
# asking for, before any diagnostic routing happens. This is intentionally a
# separate role from JevRouter above, which only decides what evidence to
# collect next once a diagnosis has already been chosen.
#
#   RequestRouter:  QUERY / DIAGNOSIS / ACTION / CHAT  (+ capability/action)
#   JevRouter:      which evidence/tool to collect next (diagnosis only)
#
# Keeping the two roles separate means a simple question
# ("what is the battery level?") never enters the iterative diagnostic loop.
# ──────────────────────────────────────────────────────────────────────────────
class RequestMode(Enum):
    QUERY = "query"
    DIAGNOSIS = "diagnosis"
    ACTION = "action"
    CHAT = "chat"


@dataclass
class QueryCapability:
    name: str
    description: str
    tool: str


@dataclass
class RequestRoute:
    mode: RequestMode
    capability: str | None = None
    confidence: float | None = None


QUERY_CAPABILITIES: dict[str, QueryCapability] = {
    "battery_level": QueryCapability(
        "battery_level",
        "Battery state of charge as a percentage (e.g. 'what is the battery level?', 'how much juice is left?').",
        "battery_state",
    ),
    "battery_voltage": QueryCapability(
        "battery_voltage",
        "Battery voltage in volts.",
        "battery_state",
    ),
    "diagnostics": QueryCapability(
        "diagnostics",
        "Standard ROS /diagnostics statuses: errors, warnings and messages.",
        "get_diagnostics",
    ),
    "ros_graph": QueryCapability(
        "ros_graph",
        "Full ROS graph summary: node, topic and service counts.",
        "inspect_graph",
    ),
    "nodes": QueryCapability(
        "nodes",
        "List of running ROS nodes.",
        "inspect_graph",
    ),
    "topics": QueryCapability(
        "topics",
        "List of discovered ROS topics and their types.",
        "inspect_graph",
    ),
    "services": QueryCapability(
        "services",
        "List of discovered ROS services and their types.",
        "inspect_graph",
    ),
    "controllers": QueryCapability(
        "controllers",
        "ros2_control controllers and hardware interfaces (controller_manager).",
        "inspect_ros2_control",
    ),
    "joint_states": QueryCapability(
        "joint_states",
        "Latest /joint_states message: named joint positions, velocities and efforts.",
        "joint_state_cached",
    ),
    "joint_state_rate": QueryCapability(
        "joint_state_rate",
        "Measured publication rate of /joint_states in Hz.",
        "sample_topic",
    ),
    "topic_snapshot": QueryCapability(
        "topic_snapshot",
        "One message from a specific ROS topic.",
        "topic_snapshot",
    ),
    "topic_statistics": QueryCapability(
        "topic_statistics",
        "Time-window statistics (min/max/mean/std/range) for a specific ROS topic.",
        "sample_topic",
    ),
    "test_history": QueryCapability(
        "test_history",
        "Recent RoboDiag health test results.",
        "query_history",
    ),
    "safety_status": QueryCapability(
        "safety_status",
        "Deterministic motion Safety Gate status.",
        "check_motion_safety",
    ),
}

REQUEST_MODE_INSTRUCTIONS = (
    "Classify the operator's request into exactly one mode. "
    "QUERY: retrieve or inspect a fact or state without investigating a problem, "
    "including checking whether there are any errors, warnings or statuses. "
    "DIAGNOSIS: a symptom, malfunction, abnormal behavior, uncertainty, or an "
    "explicit request to investigate/explain a problem. "
    "ACTION: explicitly ask to execute an operation or run a test. "
    "CHAT: general conversation or a question that does not require robot telemetry or tools."
)

REQUEST_MODE_CRITERIA: dict[str, str] = {
    "query": "Retrieve or inspect a fact or state, e.g. battery level, node list, topic rate, whether any diagnostics report errors/warnings, test history.",
    "diagnosis": "Symptom, malfunction, abnormal behavior, uncertainty, or a request to investigate/explain a problem.",
    "action": "Explicitly run an operation or test, e.g. 'run system_health', 'stop the robot'.",
    "chat": "General conversation or a question that needs no robot telemetry or tools.",
}

QUERY_CAPABILITY_INSTRUCTIONS = (
    "The operator asked a simple information question about the robot. Choose the "
    "single capability that best answers it. Prefer the most specific capability."
)

ACTION_CAPABILITY_INSTRUCTIONS = (
    "The operator asked to run an operation or test. Choose the single supported "
    "action that matches the request. Only the listed actions are available."
)


class RequestRouter:
    """System 1 request router: Jev classifies the operator's input.

    Responsibilities (kept separate from JevRouter):
      - What kind of task is this? (QUERY / DIAGNOSIS / ACTION / CHAT)
      - If QUERY: which capability answers it?
      - If ACTION: which supported action matches it?

    It never executes anything and never selects diagnostic tools; it only
    produces a structured RequestRoute. Every capability/action string it
    returns must still be re-checked against a deterministic registry before
    any tool runs.
    """

    def __init__(
        self,
        client: JevClient,
        query_capabilities: dict[str, QueryCapability] | None = None,
        action_capabilities: dict[str, QueryCapability] | None = None,
    ):
        self.client = client
        self.query_capabilities = query_capabilities or {}
        self.action_capabilities = action_capabilities or {}

    @property
    def model(self) -> str:
        return self.client.model

    def route(self, text: str) -> RequestRoute:
        answer, _model = self._ask(
            "request_mode", REQUEST_MODE_INSTRUCTIONS, REQUEST_MODE_CRITERIA, text
        )
        raw = str(answer.get("choice", "")).strip().lower()
        mode_conf = float(answer.get("confidence", 0.0) or 0.0)
        try:
            mode = RequestMode(raw)
        except ValueError:
            # Unknown labels fall back to DIAGNOSIS: the existing Jev loop is the
            # safe default for anything that may be a problem statement.
            mode = RequestMode.DIAGNOSIS

        if mode is RequestMode.QUERY and self.query_capabilities:
            capability, conf = self._resolve_capability(
                "query_capability", QUERY_CAPABILITY_INSTRUCTIONS, self.query_capabilities, text
            )
            return RequestRoute(mode=mode, capability=capability, confidence=conf)
        if mode is RequestMode.ACTION and self.action_capabilities:
            capability, conf = self._resolve_capability(
                "action_capability", ACTION_CAPABILITY_INSTRUCTIONS, self.action_capabilities, text
            )
            return RequestRoute(mode=mode, capability=capability, confidence=conf)
        return RequestRoute(mode=mode, confidence=mode_conf)

    def _resolve_capability(
        self,
        key: str,
        instructions: str,
        capabilities: dict[str, QueryCapability],
        text: str,
    ) -> tuple[str | None, float]:
        criteria = {cap.name: cap.description for cap in capabilities.values()}
        answer, _model = self._ask(key, instructions, criteria, text)
        choice = str(answer.get("choice", "")).strip()
        conf = float(answer.get("confidence", 0.0) or 0.0)
        if choice not in capabilities:
            # Never trust a label that is not in the deterministic registry.
            return None, conf
        return choice, conf

    def _ask(
        self,
        key: str,
        instructions: str,
        criteria: dict[str, str],
        text: str,
    ) -> tuple[dict[str, Any], str]:
        response = self.client.system_one(
            state={"input": text},
            questions={key: {"type": "choice", "instructions": instructions, "criteria": criteria}},
        )
        answers = response.get("answers", {})
        if key not in answers:
            raise JevError(f"Jev response missing answer for {key!r}: {response}")
        answer = answers[key]
        if answer.get("type") != "choice":
            raise JevError(f"Jev returned {answer.get('type')} for a choice question")
        return answer, response.get("model", self.client.model)

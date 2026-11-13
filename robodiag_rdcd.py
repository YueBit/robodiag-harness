#!/usr/bin/env python3
"""Robot Diagnostic Capability Description (RDCD) — standard library only.

RDCD is RoboDiag-authored, robot-specific knowledge that tells the harness
which diagnostic capabilities a given robot supports and how each capability
maps to that robot's ROS 2 interfaces. The robot itself neither generates nor
publishes RDCD; it only exposes its normal ROS 2 interfaces, and RoboDiag uses
the static RDCD to decide which diagnostics are applicable.

The core distinction this module preserves:

    NOT DECLARED   !=   DECLARED BUT MISSING

A capability that is *not declared* is ``NOT_SUPPORTED``: diagnostics for it do
not apply and must not count against overall health. A capability that *is*
declared but missing at runtime is a real finding (``MISSING``).

This module imports only the standard library so it can be unit-tested in plain
CI without ROS 2, rich, or any network access.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

SUPPORTED_SCHEMA_VERSIONS = ("0.1", "0.2")
# Retained as a convenient "newest accepted" reference; 0.1 documents are
# still accepted unchanged so existing RDCD files keep working.
SUPPORTED_SCHEMA_VERSION = "0.2"

# Capability runtime status. These are deliberately distinct from the test
# execution results (PASS / WARN / FAIL / SKIP) so that "not applicable" is
# never conflated with "applicable but could not run".
AVAILABLE = "AVAILABLE"
MISSING = "MISSING"
TYPE_MISMATCH = "TYPE_MISMATCH"
NOT_SUPPORTED = "NOT_SUPPORTED"


class RdcdError(Exception):
    """Raised when an RDCD document is missing, malformed, or unsupported."""


@dataclass(frozen=True)
class Capability:
    name: str
    topic: str
    message_type: str
    required_for_motion: bool = False
    expected_rate_hz: float | None = None
    description: str = ""


class Rdcd:
    """A validated RDCD document: robot identity plus its declared capabilities."""

    def __init__(self, robot_id: str, name: str, capabilities: dict[str, Capability]) -> None:
        self.robot_id = robot_id
        self.name = name
        self.capabilities = capabilities

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "Rdcd":
        if not isinstance(data, dict):
            raise RdcdError("RDCD document must be a mapping")

        version = data.get("schema_version")
        if str(version) not in SUPPORTED_SCHEMA_VERSIONS:
            raise RdcdError(
                f"unsupported schema_version {version!r} "
                f"(expected one of {SUPPORTED_SCHEMA_VERSIONS})"
            )

        robot = data.get("robot")
        if not isinstance(robot, dict):
            raise RdcdError("RDCD 'robot' section is missing or not a mapping")
        robot_id = robot.get("id")
        if not robot_id:
            raise RdcdError("RDCD robot.id is required")
        name = robot.get("name") or str(robot_id)

        caps_raw = data.get("capabilities")
        if not isinstance(caps_raw, dict):
            raise RdcdError("RDCD 'capabilities' section must be a mapping")

        capabilities: dict[str, Capability] = {}
        for cap_name, spec in caps_raw.items():
            if not isinstance(spec, dict):
                raise RdcdError(f"capability {cap_name!r} must be a mapping")
            topic = spec.get("topic")
            message_type = spec.get("message_type")
            if not topic or not message_type:
                raise RdcdError(
                    f"capability {cap_name!r} requires both 'topic' and 'message_type'"
                )
            required_for_motion = spec.get("required_for_motion", False)
            if not isinstance(required_for_motion, bool):
                raise RdcdError(
                    f"capability {cap_name!r}: 'required_for_motion' must be a boolean"
                )
            expected_rate_hz = spec.get("expected_rate_hz")
            if expected_rate_hz is not None:
                try:
                    expected_rate_hz = float(expected_rate_hz)
                except (TypeError, ValueError):
                    raise RdcdError(
                        f"capability {cap_name!r}: 'expected_rate_hz' must be numeric"
                    )
            capabilities[str(cap_name)] = Capability(
                str(cap_name),
                str(topic),
                str(message_type),
                required_for_motion=required_for_motion,
                expected_rate_hz=expected_rate_hz,
                description=str(spec.get("description", "")),
            )
        return cls(str(robot_id), str(name), capabilities)

    def has(self, capability: str) -> bool:
        return capability in self.capabilities

    def get(self, capability: str) -> Capability | None:
        return self.capabilities.get(capability)

    def declared(self) -> list[str]:
        return sorted(self.capabilities)

    def required_for_motion(self) -> list[str]:
        """Capabilities declared as required safety evidence for motion.

        Returns ``[]`` when the RDCD does not mark any capability as
        ``required_for_motion``; the harness then falls back to treating every
        declared capability as required (the historical generic assumption).
        """
        return sorted(
            name
            for name, cap in self.capabilities.items()
            if cap.required_for_motion
        )


def _scalar(value: str) -> Any:
    """Convert a scalar YAML value to a bool/None/int/float/str, in that order."""
    if value in ("true", "True"):
        return True
    if value in ("false", "False"):
        return False
    if value in ("null", "Null", "~"):
        return None
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        pass
    return value


def parse_rdcd_yaml(text: str) -> dict[str, Any]:
    """Parse the small, flat YAML subset used by RDCD documents.

    Supports only nested ``key: value`` mappings with scalar values and
    space-based indentation — exactly the shape of an RDCD file. Lists, quoted
    strings, anchors, and other YAML features are intentionally rejected so a
    malformed document fails loudly instead of being silently misread.
    """
    root: dict[str, Any] = {}
    stack: list[tuple[dict[str, Any], int]] = [(root, -1)]

    for lineno, raw in enumerate(text.splitlines(), 1):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        if "\t" in raw[: len(raw) - len(raw.lstrip())]:
            raise RdcdError(f"line {lineno}: tabs are not allowed for indentation")

        content = raw.strip()
        if ":" not in content:
            raise RdcdError(f"line {lineno}: expected 'key: value'")
        key, _, value = content.partition(":")
        key = key.strip()
        value = value.strip()
        if not key:
            raise RdcdError(f"line {lineno}: empty key")

        while stack and stack[-1][1] >= indent:
            stack.pop()
        if not stack:
            raise RdcdError(f"line {lineno}: unexpected indentation")
        parent = stack[-1][0]

        if value:
            parent[key] = _scalar(value)
        else:
            child: dict[str, Any] = {}
            parent[key] = child
            stack.append((child, indent))

    return root


def validate_rdcd(data: dict[str, Any]) -> list[str]:
    """Validate an RDCD document without raising; return a list of problems.

    An empty list means the document is acceptable to ``Rdcd.from_dict``. Kept
    separate so tools (e.g. an RDCD generator) can report every problem at once
    instead of stopping on the first error.
    """
    errors: list[str] = []
    if not isinstance(data, dict):
        return ["RDCD document must be a mapping"]

    version = data.get("schema_version")
    if str(version) not in SUPPORTED_SCHEMA_VERSIONS:
        errors.append(
            f"unsupported schema_version {version!r} "
            f"(expected one of {SUPPORTED_SCHEMA_VERSIONS})"
        )

    robot = data.get("robot")
    if not isinstance(robot, dict):
        errors.append("'robot' section is missing or not a mapping")
    elif not robot.get("id"):
        errors.append("robot.id is required")

    caps = data.get("capabilities")
    if not isinstance(caps, dict):
        errors.append("'capabilities' section must be a mapping")
    else:
        for name, spec in caps.items():
            if not isinstance(spec, dict):
                errors.append(f"capability {name!r} must be a mapping")
                continue
            if not spec.get("topic"):
                errors.append(f"capability {name!r} requires 'topic'")
            if not spec.get("message_type"):
                errors.append(f"capability {name!r} requires 'message_type'")
            rfm = spec.get("required_for_motion")
            if rfm is not None and not isinstance(rfm, bool):
                errors.append(f"capability {name!r}: 'required_for_motion' must be boolean")
    return errors


def load_rdcd(path: str | Path) -> Rdcd:
    """Read and validate an RDCD document from a YAML file."""
    p = Path(path).expanduser()
    try:
        text = p.read_text(encoding="utf-8")
    except OSError as exc:
        raise RdcdError(f"cannot read RDCD file {p}: {exc}") from exc
    return Rdcd.from_dict(parse_rdcd_yaml(text))


def capability_status(
    rdcd: Rdcd, capability: str, topic_types: dict[str, list[str]]
) -> str:
    """Runtime status of a declared capability against the live topic graph.

    Returns ``NOT_SUPPORTED`` when the capability is not declared (distinct from
    ``MISSING``); otherwise ``AVAILABLE`` / ``MISSING`` / ``TYPE_MISMATCH``.
    """
    spec = rdcd.get(capability)
    if spec is None:
        return NOT_SUPPORTED
    types = topic_types.get(spec.topic)
    if not types:
        return MISSING
    if spec.message_type not in types:
        return TYPE_MISMATCH
    return AVAILABLE

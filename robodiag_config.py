"""Environment / ``~/.robodiag.env`` settings resolution for the harness.

Stdlib-only (no ROS 2, no rich) so it can be imported and unit-tested anywhere.
Precedence everywhere: CLI flag > environment variable > ~/.robodiag.env > built-in default.
"""

from __future__ import annotations

import os
from pathlib import Path


def load_env(path: Path) -> dict[str, str]:
    cfg: dict[str, str] = {}
    if not path.exists():
        return cfg
    try:
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            k = k.strip()
            v = v.strip()
            # Drop trailing inline comments, then accept shell-style quotes.
            v = v.split(" #", 1)[0].strip()
            if len(v) >= 2 and v[0] in ("'", '"') and v[-1] == v[0]:
                v = v[1:-1]
            if k:
                cfg[k] = v
    except OSError:
        pass
    return cfg


def env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


# Environment-variable aliases, in priority order (first non-empty value wins).
# The LLM is any OpenAI-compatible chat/functions API, so we accept the generic
# LLM_*, the OpenAI SDK's own OPENAI_*, and the legacy DEEPSEEK_* names. Jev
# accepts JEV_* with the legacy TYPESAFE_* names still working.
_LLM_API_KEY_ENV = ("LLM_API_KEY", "OPENAI_API_KEY", "DEEPSEEK_API_KEY")
_LLM_BASE_URL_ENV = ("LLM_BASE_URL", "OPENAI_BASE_URL", "DEEPSEEK_BASE_URL")
_LLM_MODEL_ENV = ("LLM_MODEL", "OPENAI_MODEL", "DEEPSEEK_MODEL")
_JEV_API_KEY_ENV = ("JEV_API_KEY", "TYPESAFE_API_KEY")
_JEV_BASE_URL_ENV = ("JEV_BASE_URL", "TYPESAFE_BASE_URL")
_JEV_MODEL_ENV = ("JEV_MODEL", "TYPESAFE_MODEL")

_LLM_DEFAULT_BASE_URL = "https://api.deepseek.com"
_LLM_DEFAULT_MODEL = "deepseek-chat"


def _resolve_env_source(
    names: tuple[str, ...], env_file: dict[str, str]
) -> tuple[str, str | None]:
    """Return ``(value, matched_name)`` for the first non-empty alias.

    Environment variables take priority over the ``~/.robodiag.env`` file, and
    within each source the aliases are tried in the given order.
    """
    for name in names:
        value = os.environ.get(name)
        if value and value.strip():
            return value.strip(), name
    for name in names:
        value = env_file.get(name)
        if value and value.strip():
            return value.strip(), name
    return "", None


def _resolve_env(names: tuple[str, ...], env_file: dict[str, str], default: str = "") -> str:
    value, _ = _resolve_env_source(names, env_file)
    return value or default


def _llm_provider_defaults(key_source: str | None) -> tuple[str, str]:
    """Return ``(base_url, model)`` implied by the matched LLM key variable.

    ``OPENAI_API_KEY`` implies the official OpenAI endpoint and model; every
    other source (CLI flag, ``LLM_*``, legacy ``DEEPSEEK_*``) keeps the
    DeepSeek-based built-in defaults for backward compatibility.
    """
    if key_source == "OPENAI_API_KEY":
        return "https://api.openai.com/v1", "gpt-4o-mini"
    return _LLM_DEFAULT_BASE_URL, _LLM_DEFAULT_MODEL


def _normalize_llm_base_url(url: str) -> str:
    """Normalize an OpenAI-compatible base URL for the OpenAI SDK.

    The SDK appends the operation path (e.g. ``/chat/completions``) to
    ``base_url``. The official OpenAI endpoint requires the ``/v1`` version
    prefix, so accept ``https://api.openai.com`` as well as the full
    ``https://api.openai.com/v1`` form. Other providers are left untouched.
    """
    url = url.strip().rstrip("/")
    if not url:
        return url
    host = url.split("://", 1)[-1].split("/", 1)[0].lower()
    if host.endswith("openai.com") and not url.endswith("/v1"):
        return url + "/v1"
    return url


def profiles_dir() -> Path:
    return Path.home() / ".robodiag" / "profiles"


def list_profiles() -> list[str]:
    d = profiles_dir()
    return sorted(p.stem for p in d.glob("*.env")) if d.is_dir() else []


def apply_profile(name: str) -> dict[str, str]:
    """Load ``~/.robodiag/profiles/<name>.env`` (KEY=VALUE, same format as ~/.robodiag.env).

    Profile values act as per-robot defaults for ``ROBODIAG_*`` thresholds: a variable already set in the
    real environment wins. Returns the values that were applied. Raises FileNotFoundError / ValueError.
    """
    if not name or any(c in name for c in "/\\") or name.startswith("."):
        raise ValueError(f"invalid profile name: {name!r}")
    path = profiles_dir() / f"{name}.env"
    if not path.is_file():
        raise FileNotFoundError(f"profile not found: {path}")
    applied: dict[str, str] = {}
    for key, value in load_env(path).items():
        if key.startswith("ROBODIAG_") and key not in os.environ:
            os.environ[key] = value
            applied[key] = value
    return applied

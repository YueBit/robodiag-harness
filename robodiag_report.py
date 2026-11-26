"""Diagnostic report export and history trend analysis.

Stdlib-only (no ROS 2, no rich). Takes the data the harness already stores
(``HistoryStore`` rows, a session conclusion) and renders Markdown / HTML, or
looks for degradation in the test history.
"""

from __future__ import annotations

import html
import json
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SEVERITY = {"PASS": 0, "NOT_SUPPORTED": 0, "SKIP": 1, "WARN": 2, "FAIL": 3}
_BADGE = {"PASS": "✅", "NOT_SUPPORTED": "➖", "SKIP": "⏭️", "WARN": "⚠️", "FAIL": "❌"}
_COLOR = {"PASS": "#1b8a3a", "NOT_SUPPORTED": "#777", "SKIP": "#777", "WARN": "#b36b00", "FAIL": "#c62828"}


# ──────────────────────────────────────────────────────────────────────────────
# Trend analysis
# ──────────────────────────────────────────────────────────────────────────────
def analyze_trend(rows: list[dict[str, Any]], streak: int = 3, window: int = 10) -> list[dict[str, Any]]:
    """Find degrading tests in history ``rows`` (any order; need ``test_id``/``result``/``started_at``).

    Alert kinds:
      * ``fail_streak``  - the last ``streak`` runs of a test all FAILed
      * ``regression``   - the latest run is worse than the previous one (e.g. PASS -> FAIL)
      * ``worsening``    - the failure/warning rate of the latest half-window is clearly above the earlier half
      * ``slowing``      - median duration of the latest half-window is >2x the earlier half (min 500 ms)
    """
    by_test: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for r in sorted(rows, key=lambda x: (x.get("started_at", ""), x.get("id", 0))):
        by_test[r["test_id"]].append(r)

    alerts: list[dict[str, Any]] = []
    for test_id, runs in by_test.items():
        runs = runs[-max(window, streak):]
        results = [r["result"] for r in runs]
        sev = [SEVERITY.get(x, 3) for x in results]
        last = results[-1]

        if len(runs) >= streak and all(x == "FAIL" for x in results[-streak:]):
            alerts.append({"test_id": test_id, "kind": "fail_streak", "level": "FAIL",
                           "message": f"{test_id} failed the last {streak} runs in a row"})
        elif len(sev) >= 2 and sev[-1] > sev[-2] and sev[-1] >= 2:
            alerts.append({"test_id": test_id, "kind": "regression", "level": last,
                           "message": f"{test_id} regressed: {results[-2]} -> {last}"})

        if len(runs) >= 6:
            half = len(runs) // 2
            early, late = sev[:half], sev[half:]
            e_rate = sum(1 for x in early if x >= 2) / len(early)
            l_rate = sum(1 for x in late if x >= 2) / len(late)
            if l_rate - e_rate >= 0.34:
                alerts.append({"test_id": test_id, "kind": "worsening", "level": "WARN",
                               "message": f"{test_id} problem rate rose from {e_rate:.0%} to {l_rate:.0%}"})
            d_early = sorted(r.get("duration_ms", 0) for r in runs[:half])
            d_late = sorted(r.get("duration_ms", 0) for r in runs[half:])
            m_early, m_late = d_early[len(d_early) // 2], d_late[len(d_late) // 2]
            if m_late >= 500 and m_early > 0 and m_late > 2 * m_early:
                alerts.append({"test_id": test_id, "kind": "slowing", "level": "WARN",
                               "message": f"{test_id} median duration grew {m_early} ms -> {m_late} ms"})
    alerts.sort(key=lambda a: (-SEVERITY.get(a["level"], 3), a["test_id"]))
    return alerts


def format_trend(rows: list[dict[str, Any]], alerts: list[dict[str, Any]]) -> str:
    """Plain-text trend summary: per-test result strip (oldest -> newest) plus alerts."""
    by_test: dict[str, list[str]] = defaultdict(list)
    for r in sorted(rows, key=lambda x: (x.get("started_at", ""), x.get("id", 0))):
        by_test[r["test_id"]].append(r["result"])
    if not by_test:
        return "No test history yet."
    glyph = {"PASS": ".", "NOT_SUPPORTED": "-", "SKIP": "s", "WARN": "W", "FAIL": "F"}
    lines = ["Result history per test (oldest → newest; . pass  W warn  F fail  s skip):"]
    for test_id in sorted(by_test):
        strip = "".join(glyph.get(x, "?") for x in by_test[test_id][-20:])
        lines.append(f"  {test_id:<28} {strip}")
    lines.append("")
    if alerts:
        lines.append("Alerts:")
        lines += [f"  [{a['level']}] {a['message']}" for a in alerts]
    else:
        lines.append("No degradation detected.")
    return "\n".join(lines)


# ──────────────────────────────────────────────────────────────────────────────
# Report rendering
# ──────────────────────────────────────────────────────────────────────────────
def latest_per_test(rows: list[dict[str, Any]], since: str | None = None) -> list[dict[str, Any]]:
    """Newest run of each test; ``since`` (ISO timestamp) drops runs that started earlier (e.g. older sessions)."""
    latest: dict[str, dict[str, Any]] = {}
    for r in sorted(rows, key=lambda x: (x.get("started_at", ""), x.get("id", 0))):
        if since and r.get("started_at", "") < since:
            continue
        latest[r["test_id"]] = r
    return list(latest.values())


def _short(value: Any, limit: int = 300) -> str:
    text = value if isinstance(value, str) else json.dumps(value, ensure_ascii=False, default=str)
    return text if len(text) <= limit else text[: limit - 1] + "…"


def _overall(results: list[dict[str, Any]]) -> str:
    worst = max((SEVERITY.get(r["result"], 3) for r in results), default=0)
    return {0: "PASS", 1: "SKIP", 2: "WARN", 3: "FAIL"}[worst]


def build_report_data(
    results: list[dict[str, Any]],
    conclusion: str | None = None,
    meta: dict[str, Any] | None = None,
    host: dict[str, Any] | None = None,
    alerts: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """``results`` are full test-result dicts (``test_id``, ``result``, ``summary``, ``evidence``...)."""
    counts: dict[str, int] = defaultdict(int)
    for r in results:
        counts[r["result"]] += 1
    return {
        "generated_at": datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds"),
        "meta": meta or {},
        "overall": _overall(results),
        "counts": dict(counts),
        "results": results,
        "conclusion": conclusion or "",
        "host": host or {},
        "alerts": alerts or [],
    }


def render_markdown(data: dict[str, Any]) -> str:
    out = ["# RoboDiag diagnostic report", "", f"- Generated: {data['generated_at']}",
           f"- Overall: {_BADGE.get(data['overall'], '')} **{data['overall']}**"]
    for k, v in data["meta"].items():
        out.append(f"- {k}: {v}")
    out.append("- Counts: " + (", ".join(f"{k} {v}" for k, v in sorted(data["counts"].items())) or "none"))
    if data["conclusion"]:
        out += ["", "## Diagnosis", "", data["conclusion"].strip()]
    if data["alerts"]:
        out += ["", "## Trend alerts", ""] + [f"- **{a['level']}** {a['message']}" for a in data["alerts"]]
    out += ["", "## Test results", "", "| Test | When | Result | Duration | Summary |", "|---|---|---|---|---|"]
    for r in data["results"]:
        summary = r["summary"].replace("|", "\\|").replace("\n", " ")
        out.append(f"| {r['test_id']} | {r.get('started_at', '')} | {_BADGE.get(r['result'], '')} {r['result']} | {r.get('duration_ms', 0)} ms | {summary} |")
    ev_blocks = []
    for r in data["results"]:
        if r.get("evidence"):
            ev_blocks.append(f"### {r['test_id']}\n")
            for e in r["evidence"]:
                ev_blocks.append(f"- `{e.get('source', '?')}` · {e.get('metric', '?')} ({e.get('quality', 'measured')}): "
                                 f"`{_short(e.get('value'))}`")
            ev_blocks.append("")
    if ev_blocks:
        out += ["", "## Evidence", ""] + ev_blocks
    if data["host"]:
        out += ["## Host resources", "", "```json", json.dumps(data["host"], indent=2, ensure_ascii=False), "```", ""]
    return "\n".join(out).rstrip() + "\n"


def render_html(data: dict[str, Any]) -> str:
    esc = html.escape
    rows = "".join(
        f"<tr><td>{esc(r['test_id'])}</td><td>{esc(str(r.get('started_at', '')))}</td>"
        f"<td style='color:{_COLOR.get(r['result'], '#000')};font-weight:600'>{esc(r['result'])}</td>"
        f"<td>{r.get('duration_ms', 0)} ms</td><td>{esc(r['summary'])}</td></tr>"
        for r in data["results"]
    )
    evidence = ""
    for r in data["results"]:
        if r.get("evidence"):
            items = "".join(
                f"<li><code>{esc(str(e.get('source', '?')))}</code> · {esc(str(e.get('metric', '?')))}: "
                f"<code>{esc(_short(e.get('value')))}</code></li>" for e in r["evidence"])
            evidence += f"<h3>{esc(r['test_id'])}</h3><ul>{items}</ul>"
    alerts = "".join(f"<li><b>{esc(a['level'])}</b> {esc(a['message'])}</li>" for a in data["alerts"])
    meta = "".join(f"<li>{esc(str(k))}: {esc(str(v))}</li>" for k, v in data["meta"].items())
    conclusion = f"<h2>Diagnosis</h2><pre style='white-space:pre-wrap'>{esc(data['conclusion'].strip())}</pre>" if data["conclusion"] else ""
    host = (f"<h2>Host resources</h2><pre>{esc(json.dumps(data['host'], indent=2, ensure_ascii=False))}</pre>"
            if data["host"] else "")
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><title>RoboDiag report</title>
<style>body{{font-family:system-ui,sans-serif;max-width:960px;margin:2rem auto;padding:0 1rem;color:#222}}
table{{border-collapse:collapse;width:100%}}td,th{{border:1px solid #ddd;padding:6px 8px;text-align:left;vertical-align:top}}
th{{background:#f4f4f4}}code{{background:#f4f4f4;padding:1px 4px;border-radius:3px}}</style></head><body>
<h1>RoboDiag diagnostic report</h1>
<ul><li>Generated: {esc(data['generated_at'])}</li>
<li>Overall: <b style="color:{_COLOR.get(data['overall'], '#000')}">{esc(data['overall'])}</b></li>{meta}</ul>
{conclusion}
{f'<h2>Trend alerts</h2><ul>{alerts}</ul>' if alerts else ''}
<h2>Test results</h2>
<table><tr><th>Test</th><th>When</th><th>Result</th><th>Duration</th><th>Summary</th></tr>{rows}</table>
{f'<h2>Evidence</h2>{evidence}' if evidence else ''}
{host}
</body></html>
"""


def write_report(data: dict[str, Any], path: str | Path | None = None, fmt: str = "md") -> Path:
    """Write the report; ``fmt`` is ``md`` or ``html`` (a path suffix overrides it). Returns the path."""
    fmt = fmt.lower()
    if path is None:
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        path = Path.home() / ".robodiag" / "reports" / f"report-{stamp}.{'html' if fmt == 'html' else 'md'}"
    path = Path(path).expanduser()
    if path.suffix.lower() in (".html", ".htm"):
        fmt = "html"
    elif path.suffix.lower() in (".md", ".markdown"):
        fmt = "md"
    if fmt not in ("md", "html"):
        raise ValueError("format must be 'md' or 'html'")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(render_html(data) if fmt == "html" else render_markdown(data), encoding="utf-8")
    return path

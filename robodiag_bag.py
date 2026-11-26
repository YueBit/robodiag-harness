"""Offline health analysis of a rosbag2 recording (no ROS 2 needed).

A rosbag2 ``.db3`` file is plain SQLite (``topics`` + ``messages`` tables), so
topic rates, gaps and jitter can be checked from the message timestamps alone,
without deserialising any message. Compressed bags and MCAP storage are
reported as unsupported rather than guessed at.
"""

from __future__ import annotations

import sqlite3
import statistics
from pathlib import Path
from typing import Any

SEVERITY = {"PASS": 0, "WARN": 2, "FAIL": 3}


class BagError(Exception):
    pass


def find_db3(path: str | Path) -> list[Path]:
    p = Path(path).expanduser()
    if p.is_file():
        if p.suffix == ".mcap":
            raise BagError("MCAP storage is not supported offline; convert with `ros2 bag convert` to sqlite3")
        if p.suffix == ".zstd" or p.name.endswith(".db3.zstd"):
            raise BagError("compressed bag; decompress it first (`ros2 bag reindex`/zstd -d)")
        return [p]
    if p.is_dir():
        files = sorted(p.glob("*.db3"))
        if files:
            return files
        if list(p.glob("*.mcap")):
            raise BagError("MCAP storage is not supported offline; convert with `ros2 bag convert` to sqlite3")
        if list(p.glob("*.zstd")):
            raise BagError("compressed bag; decompress it first")
    raise BagError(f"no rosbag2 sqlite3 (.db3) data found at {p}")


def analyze_bag(
    path: str | Path,
    min_rate_hz: float = 1.0,
    gap_factor: float = 5.0,
    jitter_warn: float = 0.5,
    expected_topics: list[str] | None = None,
) -> dict[str, Any]:
    """Per-topic rate / gap / jitter checks over a bag.

    A topic is flagged WARN when it has a gap longer than ``gap_factor`` x its median period, or its period
    coefficient of variation exceeds ``jitter_warn``; FAIL when it is below ``min_rate_hz`` and has
    fewer than 2 messages, or is listed in ``expected_topics`` but absent.
    """
    files = find_db3(path)
    topics: dict[str, dict[str, Any]] = {}
    stamps: dict[str, list[int]] = {}
    for f in files:
        con = sqlite3.connect(f"file:{f}?mode=ro", uri=True)
        try:
            for tid, name, ttype in con.execute("SELECT id, name, type FROM topics"):
                topics.setdefault(name, {"type": ttype})
                rows = con.execute("SELECT timestamp FROM messages WHERE topic_id = ? ORDER BY timestamp", (tid,))
                stamps.setdefault(name, []).extend(r[0] for r in rows)
        except sqlite3.DatabaseError as exc:
            raise BagError(f"{f.name}: not a rosbag2 sqlite3 file ({exc})") from exc
        finally:
            con.close()

    all_ts = [t for v in stamps.values() for t in v]
    if not all_ts:
        raise BagError("bag contains no messages")
    t0, t1 = min(all_ts), max(all_ts)
    duration_s = (t1 - t0) / 1e9

    report_topics = []
    worst = 0
    for name in sorted(topics):
        ts = sorted(stamps.get(name, []))
        entry: dict[str, Any] = {"topic": name, "type": topics[name]["type"], "messages": len(ts), "issues": []}
        level = "PASS"
        if len(ts) < 2:
            entry["rate_hz"] = 0.0
            if len(ts) == 0 or min_rate_hz > 0:
                entry["issues"].append(f"only {len(ts)} message(s) in a {duration_s:.1f}s bag")
                level = "FAIL" if len(ts) == 0 else "WARN"
        else:
            periods = [(b - a) / 1e9 for a, b in zip(ts, ts[1:])]
            span = (ts[-1] - ts[0]) / 1e9
            rate = (len(ts) - 1) / span if span > 0 else 0.0
            median = statistics.median(periods)
            mean = statistics.fmean(periods)
            cv = (statistics.pstdev(periods) / mean) if mean > 0 else 0.0
            max_gap = max(periods)
            entry.update(rate_hz=round(rate, 3), median_period_ms=round(median * 1e3, 3),
                         max_gap_s=round(max_gap, 3), jitter_cv=round(cv, 3),
                         first_offset_s=round((ts[0] - t0) / 1e9, 3), last_offset_s=round((t1 - ts[-1]) / 1e9, 3))
            if rate < min_rate_hz:
                entry["issues"].append(f"rate {rate:.2f} Hz below {min_rate_hz} Hz")
                level = "WARN"
            if median > 0 and max_gap > gap_factor * median and max_gap > 0.2:
                entry["issues"].append(f"gap of {max_gap:.2f}s (median period {median * 1e3:.1f} ms)")
                level = "WARN"
            if cv > jitter_warn:
                entry["issues"].append(f"high jitter (CV {cv:.2f})")
                level = "WARN"
            if (t1 - ts[-1]) / 1e9 > max(1.0, 10 * median):
                entry["issues"].append(f"stopped {(t1 - ts[-1]) / 1e9:.1f}s before the end of the recording")
                level = "WARN"
        entry["result"] = level
        worst = max(worst, SEVERITY[level])
        report_topics.append(entry)

    missing = [t for t in (expected_topics or []) if t not in topics]
    for t in missing:
        report_topics.append({"topic": t, "type": "?", "messages": 0, "result": "FAIL",
                              "issues": ["expected topic is not in the bag"]})
        worst = 3
    overall = {0: "PASS", 2: "WARN", 3: "FAIL"}[worst]
    return {"ok": True, "path": str(Path(path).expanduser()), "duration_s": round(duration_s, 3),
            "topics": report_topics, "overall": overall,
            "summary": f"{len(topics)} topics over {duration_s:.1f}s: "
                       f"{sum(1 for t in report_topics if t['result'] == 'FAIL')} FAIL, "
                       f"{sum(1 for t in report_topics if t['result'] == 'WARN')} WARN"}


def format_bag_report(res: dict[str, Any]) -> str:
    lines = [f"Bag {res['path']}", f"{res['overall']} — {res['summary']}", ""]
    lines.append(f"{'topic':<34}{'msgs':>7}{'Hz':>9}{'max gap':>9}  result")
    for t in res["topics"]:
        lines.append(f"{t['topic'][:33]:<34}{t['messages']:>7}{t.get('rate_hz', 0):>9}{t.get('max_gap_s', '-'):>9}  {t['result']}")
        lines += [f"    · {i}" for i in t["issues"]]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    import argparse

    ap = argparse.ArgumentParser(description="Offline rosbag2 (sqlite3) health check; no ROS 2 required")
    ap.add_argument("path", help="bag directory or .db3 file")
    ap.add_argument("--expect", action="append", default=[], help="topic that must be present (repeatable)")
    ap.add_argument("--min-rate", type=float, default=1.0)
    args = ap.parse_args(argv)
    try:
        res = analyze_bag(args.path, min_rate_hz=args.min_rate, expected_topics=args.expect)
    except (BagError, OSError) as exc:
        print(f"robodiag-bag: {exc}")
        return 2
    print(format_bag_report(res))
    return {"PASS": 0, "WARN": 1, "FAIL": 3}[res["overall"]]


if __name__ == "__main__":
    raise SystemExit(main())

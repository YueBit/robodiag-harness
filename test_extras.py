"""Tests for report export, trend alerts, offline bag analysis, TF analysis and profiles."""
import os
import sqlite3
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import robodiag_bag as bag
import robodiag_config as cfg
import robodiag_core as core
import robodiag_report as rep
from robodiag_jev import TestResult


def _row(i, test_id, result, dur=100):
    return {"id": i, "started_at": f"2026-01-01T00:00:{i:02d}", "test_id": test_id, "result": result,
            "summary": f"{test_id} {result}", "duration_ms": dur, "evidence": []}


class TestTrend(unittest.TestCase):
    def test_fail_streak_and_regression(self):
        rows = [_row(i, "imu", "FAIL") for i in range(1, 4)] + [_row(10, "gps", "PASS"), _row(11, "gps", "WARN")]
        kinds = {(a["test_id"], a["kind"]) for a in rep.analyze_trend(rows)}
        self.assertIn(("imu", "fail_streak"), kinds)
        self.assertIn(("gps", "regression"), kinds)

    def test_worsening_and_slowing(self):
        rows = [_row(i, "t", "PASS", 100) for i in range(1, 4)] + [_row(i, "t", "WARN", 900) for i in range(4, 8)]
        kinds = {a["kind"] for a in rep.analyze_trend(rows)}
        self.assertIn("worsening", kinds)
        self.assertIn("slowing", kinds)

    def test_healthy_history_has_no_alerts(self):
        rows = [_row(i, "t", "PASS") for i in range(1, 9)]
        self.assertEqual(rep.analyze_trend(rows), [])
        self.assertIn("No degradation", rep.format_trend(rows, []))


class TestReport(unittest.TestCase):
    def _data(self):
        results = [_row(1, "imu", "PASS"), _row(2, "battery", "FAIL")]
        results[1]["evidence"] = [{"source": "/battery", "metric": "pct", "value": 0.02, "quality": "measured"}]
        results[0]["summary"] = "a|b <x>"
        return rep.build_report_data(results, conclusion="Battery is low", meta={"RDCD": "generic"},
                                     alerts=[{"level": "WARN", "message": "battery regressed"}])

    def test_overall_is_worst_result(self):
        self.assertEqual(self._data()["overall"], "FAIL")

    def test_markdown_escapes_table_and_lists_evidence(self):
        md = rep.render_markdown(self._data())
        self.assertIn("a\\|b <x>", md)
        self.assertIn("Battery is low", md)
        self.assertIn("`/battery`", md)

    def test_html_escapes_and_write_picks_format_from_suffix(self):
        html_doc = rep.render_html(self._data())
        self.assertIn("a|b &lt;x&gt;", html_doc)
        with tempfile.TemporaryDirectory() as d:
            out = rep.write_report(self._data(), Path(d) / "r.html", "md")
            self.assertTrue(out.read_text().startswith("<!doctype html>"))
            with self.assertRaises(ValueError):
                rep.write_report(self._data(), Path(d) / "r.txt", "pdf")

    def test_history_full_returns_evidence_oldest_first(self):
        with tempfile.TemporaryDirectory() as d:
            h = core.HistoryStore(Path(d) / "h.db")
            for n in ("a", "b"):
                h.add(TestResult(n, "PASS", "ok", [{"source": "s", "metric": "m", "value": 1}], "2026-01-01T00:00:00", 5))
            rows = h.full(10)
            self.assertEqual([r["test_id"] for r in rows], ["a", "b"])
            self.assertEqual(rows[0]["evidence"][0]["metric"], "m")


def _make_bag(path, topics):
    con = sqlite3.connect(path)
    con.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY, name TEXT, type TEXT, serialization_format TEXT, offered_qos_profiles TEXT)")
    con.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY, topic_id INTEGER, timestamp INTEGER, data BLOB)")
    for tid, (name, stamps) in enumerate(topics.items(), 1):
        con.execute("INSERT INTO topics VALUES (?, ?, 'x/msg/T', 'cdr', '')", (tid, name))
        con.executemany("INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, x'00')", [(tid, t) for t in stamps])
    con.commit()
    con.close()


class TestBag(unittest.TestCase):
    def test_detects_gap_jitter_and_missing_topic(self):
        sec = 10**9
        steady = [i * sec // 10 for i in range(100)]                       # 10 Hz for 10 s
        gappy = [i * sec // 10 for i in range(30)] + [i * sec // 10 for i in range(70, 100)]  # 4 s hole
        with tempfile.TemporaryDirectory() as d:
            db = Path(d) / "b_0.db3"
            _make_bag(db, {"/imu": steady, "/odom": gappy})
            res = bag.analyze_bag(d, expected_topics=["/scan"])
        by = {t["topic"]: t for t in res["topics"]}
        self.assertEqual(by["/imu"]["result"], "PASS")
        self.assertAlmostEqual(by["/imu"]["rate_hz"], 10.0, places=1)
        self.assertEqual(by["/odom"]["result"], "WARN")
        self.assertTrue(any("gap" in i for i in by["/odom"]["issues"]))
        self.assertEqual(by["/scan"]["result"], "FAIL")
        self.assertEqual(res["overall"], "FAIL")
        self.assertIn("/imu", bag.format_bag_report(res))

    def test_rejects_unsupported_and_empty(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "x.mcap").write_bytes(b"")
            with self.assertRaises(bag.BagError):
                bag.analyze_bag(d)
        with tempfile.TemporaryDirectory() as d:
            _make_bag(Path(d) / "e.db3", {"/a": []})
            with self.assertRaises(bag.BagError):
                bag.analyze_bag(d)
        with self.assertRaises(bag.BagError):
            bag.analyze_bag("/nonexistent/bag")


def _e(parent, child, static=False, age=0.1, off=0.0):
    return {"parent": parent, "child": child, "static": static, "count": 5, "last_age_s": age, "stamp_offset_s": off}


class TestTf(unittest.TestCase):
    def test_healthy_tree_passes(self):
        res, summary, _ = core.analyze_tf({"edges": [_e("map", "odom"), _e("odom", "base_link"), _e("base_link", "lidar", True)]})
        self.assertEqual(res, "PASS")
        self.assertIn("map", summary)

    def test_disconnected_duplicate_parent_and_cycle_fail(self):
        res, summary, _ = core.analyze_tf({"edges": [_e("map", "odom"), _e("base_link", "lidar")]})
        self.assertEqual(res, "FAIL")
        self.assertIn("disconnected", summary)
        res, summary, _ = core.analyze_tf({"edges": [_e("a", "c"), _e("b", "c")]})
        self.assertEqual(res, "FAIL")
        self.assertIn("multiple parents", summary)
        res, _, _ = core.analyze_tf({"edges": [_e("a", "b"), _e("b", "a")]})
        self.assertEqual(res, "FAIL")

    def test_stale_and_clock_skew_warn(self):
        res, summary, _ = core.analyze_tf({"edges": [_e("map", "odom", age=9.0)]})
        self.assertEqual((res, "stale" in summary), ("WARN", True))
        res, summary, _ = core.analyze_tf({"edges": [_e("map", "odom", off=-3.2)]})
        self.assertEqual((res, "clock" in summary), ("WARN", True))
        res, _, _ = core.analyze_tf({"edges": [_e("map", "odom", True, off=-3.2)]})  # static stamps are exempt
        self.assertEqual(res, "PASS")

    def test_no_tf_is_skipped_by_runner_test(self):
        with tempfile.TemporaryDirectory() as d:
            node = mock.Mock()
            node.sample_tf.return_value = {"ok": False, "error": "no transforms"}
            runner = core.TestRunner(node, core.HistoryStore(Path(d) / "h.db"))
            self.assertEqual(runner.run("tf_health")["result"], "SKIP")
            node.sample_tf.return_value = {"ok": True, "edges": [_e("map", "odom")]}
            self.assertEqual(runner.run("tf_health")["result"], "PASS")


class TestProfiles(unittest.TestCase):
    def test_profile_sets_defaults_but_real_env_wins(self):
        with tempfile.TemporaryDirectory() as d, mock.patch.object(cfg.Path, "home", return_value=Path(d)):
            pdir = Path(d) / ".robodiag" / "profiles"
            pdir.mkdir(parents=True)
            (pdir / "arm1.env").write_text("ROBODIAG_MIN_BATTERY_PCT=0.3\nROBODIAG_MAX_DIAG_AGE_S=9\nOTHER=1\n")
            with mock.patch.dict(os.environ, {"ROBODIAG_MAX_DIAG_AGE_S": "2"}, clear=True):
                applied = cfg.apply_profile("arm1")
                self.assertEqual(applied, {"ROBODIAG_MIN_BATTERY_PCT": "0.3"})
                self.assertEqual(os.environ["ROBODIAG_MAX_DIAG_AGE_S"], "2")
                self.assertNotIn("OTHER", os.environ)
            self.assertEqual(cfg.list_profiles(), ["arm1"])
            with self.assertRaises(FileNotFoundError):
                cfg.apply_profile("nope")
            for bad in ("../x", ".hidden", ""):
                with self.assertRaises(ValueError):
                    cfg.apply_profile(bad)


if __name__ == "__main__":
    unittest.main()

import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import robodiag_config as cfg


class TestConfig(unittest.TestCase):
    def test_load_env_parses_quotes_and_comments(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / ".robodiag.env"
            p.write_text("# c\nLLM_API_KEY='sk-1'\nJEV_MODEL=m # inline\n\nbad line\n", encoding="utf-8")
            self.assertEqual(cfg.load_env(p), {"LLM_API_KEY": "sk-1", "JEV_MODEL": "m"})
        self.assertEqual(cfg.load_env(Path("/nonexistent/.robodiag.env")), {})

    def test_environment_beats_env_file_and_alias_order_holds(self):
        file_cfg = {"LLM_API_KEY": "from-file", "DEEPSEEK_API_KEY": "legacy-file"}
        with mock.patch.dict(os.environ, {"OPENAI_API_KEY": "from-env"}, clear=True):
            self.assertEqual(cfg._resolve_env_source(cfg._LLM_API_KEY_ENV, file_cfg), ("from-env", "OPENAI_API_KEY"))
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(cfg._resolve_env_source(cfg._LLM_API_KEY_ENV, file_cfg), ("from-file", "LLM_API_KEY"))
            self.assertEqual(cfg._resolve_env(cfg._JEV_API_KEY_ENV, {}, "dflt"), "dflt")

    def test_openai_key_implies_openai_defaults(self):
        self.assertEqual(cfg._llm_provider_defaults("OPENAI_API_KEY"), ("https://api.openai.com/v1", "gpt-4o-mini"))
        self.assertEqual(cfg._llm_provider_defaults("LLM_API_KEY"), ("https://api.deepseek.com", "deepseek-chat"))

    def test_openai_base_url_gets_v1(self):
        self.assertEqual(cfg._normalize_llm_base_url("https://api.openai.com/"), "https://api.openai.com/v1")
        self.assertEqual(cfg._normalize_llm_base_url("https://api.deepseek.com"), "https://api.deepseek.com")

    def test_env_float_falls_back(self):
        with mock.patch.dict(os.environ, {"X_F": "2.5", "X_BAD": "nope"}):
            self.assertEqual(cfg.env_float("X_F", 1.0), 2.5)
            self.assertEqual(cfg.env_float("X_BAD", 1.0), 1.0)


if __name__ == "__main__":
    unittest.main()

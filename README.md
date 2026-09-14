# RoboDiag Harness

**An LLM-powered diagnostic agent for ROS 2 robots.**
*Graph → Evidence → Test → Diagnosis.*

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)
![Python](https://img.shields.io/badge/Python-3.10-blue)

RoboDiag Harness is a terminal diagnostic agent for ROS 2 robots. It inspects the
live ROS graph, collects standard `/diagnostics`, dynamically snapshots and
samples arbitrary topics, runs deterministic read-only health tests, persists
every run to SQLite, and drives an OpenAI-compatible LLM (DeepSeek by default)
through a function-calling loop that explains *what is wrong and why*.

The core rule:

> **The LLM may request evidence and tests, but deterministic code decides
> whether a write or motion operation is permitted.**

---

## ⚠️ Safety

The included emergency stop is a **software-level fallback**. It publishes a zero
`geometry_msgs/Twist` to a configurable topic and can optionally call a
`std_srvs/Trigger` service. It is **not** a certified hardware E-stop.

The v0.1 test catalog is entirely read-only and never intentionally moves the
robot. When motion-related extensions are added, the deterministic Safety Gate —
never the LLM — will decide whether they are permitted.

---

## Features

- **Inspect the ROS graph** — nodes, topics, services and their types.
- **Standard diagnostics** — collect `/diagnostics` (`diagnostic_msgs/DiagnosticArray`).
- **ros2_control introspection** — controllers, hardware components and interfaces
  through `controller_manager` services, when present.
- **Dynamic topic probing** — snapshot/sample *any* discovered topic by loading its
  message type at runtime; nothing is hard-coded.
- **Time-window sampling** — compute `min/max/mean/std/range` per numeric field to
  diagnose jitter, drift, dropouts and intermittency.
- **Deterministic health tests** — structured `PASS / WARN / FAIL / SKIP` results,
  persisted to SQLite for comparison over time.
- **Safety Gate** — fail-closed, deterministic preconditions for motion.
- **AI diagnostic agent** — OpenAI-compatible function calling, DeepSeek by default,
  with a strict tool contract.
- **Terminal UX** — `rich` rendering, animated banner, and a live `/`-command
  completion menu (`prompt_toolkit`, with a `readline` fallback).

---

## How it works

```
      natural language                         slash commands
            │                                        │
            ▼                                        ▼
   ┌──────────────────┐                    ┌──────────────────┐
   │   Agent loop     │                    │   REPL (rich)    │
   │  LLM function    │                    │  direct dispatch │
   │  calling, ≤10    │                    └────────┬─────────┘
   └────────┬─────────┘                             │
            │ tool calls                    same deterministic tools
            ▼                                        ▼
   ┌──────────────────────────────────────────────────────────┐
   │                 Deterministic tool layer                 │
   │   HarnessNode (rclpy)  ·  TestRunner  ·  HistoryStore     │
   └───────────┬───────────────────────┬──────────────────────┘
               │                       │
       ROS 2 graph / topics       SQLite test history
               │
       ┌───────┴────────┐
       │  Safety Gate   │  fail-closed, no LLM in the loop
       │  diagnostics / │  checks freshness + battery + joints
       │  battery /     │
       │  joint states  │
       └────────────────┘
```

Every tool returns structured JSON. Errors are converted to
`{"ok": false, "error": ...}` and never thrown at the model, so a single failure
cannot abort a diagnostic session.

### How is this different from `ros2 doctor`?

`ros2 doctor` inspects the local ROS installation and its configuration. RoboDiag
Harness inspects the **live running graph and robot telemetry**, samples topics
over time, runs deterministic health tests, stores history, and adds an LLM agent
that can decide *which* evidence to collect next.

---

## Requirements

- ROS 2 (tested on Humble; mostly distro-agnostic), Python 3.10
- `rich` — `python3 -m pip install --user rich`
- optional `prompt_toolkit` for the live `/`-command completion menu
  (falls back to `readline` when absent) — `python3 -m pip install --user prompt_toolkit`
- optional `openai` for the AI agent — `python3 -m pip install --user openai`
- for ros2_control checks: `ros-humble-controller-manager-msgs`

---

## Install

`pip install` provides the `robodiag` console command. **rclpy and the ROS
message packages are not on PyPI** — they come from your ROS 2 distribution, so
source ROS before running.

```bash
# from PyPI (once published)
pip install robodiag-harness             # core (rich only)
pip install "robodiag-harness[all]"      # + prompt_toolkit completion + openai agent

# from source / editable
git clone https://github.com/YueBit/robodiag-harness.git
cd robodiag-harness
pip install -e ".[all]"
```

> ROS 2 Humble images ship setuptools < 61 and may have no PyPI access. In that
> case build with the system setuptools instead of an isolated one:
>
> ```bash
> pip install --user --no-build-isolation ".[all]"
> ```

## Quick start

In a shell where ROS is sourced, the installed `robodiag` command is enough:

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash       # your robot workspace, if any
robodiag
```

The repo also ships a **bash launcher** (`./robodiag`) that sources ROS 2 and an
optional `ROBODIAG_WS` overlay automatically:

```bash
./robodiag
./robodiag --cmd-vel-topic /cmd_vel
ROBODIAG_WS=~/robot_ws ./robodiag --estop-service /emergency_stop
```

The launcher expects ROS under `/opt/ros`: if `ROS_DISTRO` is set it uses exactly
that distro and **fails loudly** if its `setup.bash` is missing; otherwise it
defaults to Humble. If `ROBODIAG_WS` is set but has no built `install/setup.bash`,
it also exits with an error instead of silently continuing without your custom
messages/services. The launcher is symlink-safe, so
`ln -s …/robodiag-harness/robodiag ~/.local/bin/robodiag` works too.

You can also run the script directly:

```bash
python3 robodiag_ros2.py
```

> The pip console command and the repo's bash launcher are both named `robodiag`.
> Only the bash launcher auto-sources ROS; the console command assumes ROS is
> already sourced.

### CLI flags

| Flag | Default | Purpose |
|---|---|---|
| `--controller-manager` | `/controller_manager` | controller_manager node namespace |
| `--cmd-vel-topic` | `/cmd_vel` | topic used by the software stop |
| `--estop-service` | *(none)* | optional `std_srvs/Trigger` E-stop service |
| `--db` | `~/.robodiag_ros2.db` | SQLite history path |
| `--model` | `deepseek-chat` | LLM model; overrides `DEEPSEEK_MODEL` |
| `--base-url` | `https://api.deepseek.com` | OpenAI-compatible base URL |
| `--api-key` | *(none)* | LLM API key (visible in `ps`/shell history) |
| `--api-key-file` | *(none)* | read the API key from a file (safer) |

---

## REPL commands

```
/graph                         ROS graph summary
/diagnostics                   Standard /diagnostics status
/control                       ros2_control status
/check                         Composite read-only health check
/topic <topic>                 Snapshot one message from a topic
/sample <topic> [sec] [Hz]     Sample a topic over time and compute statistics
/tests                         Test catalog
/run <test_id>                 Run a deterministic test (all read-only in v0.1)
/history [n]                   Recent n test runs
/safety                        Show the motion Safety Gate
/stop                          Software stop: zero cmd_vel + optional Trigger service
/help /quit
```

Any other input goes to the **AI diagnostic agent** (requires an API key).

---

## AI diagnostic agent

The agent runs a function-calling loop of at most 10 rounds. Each tool result is
fed back as structured JSON, and the final answer follows a fixed shape:
**symptom → evidence → root cause → recommendations**.

### Tools and risk layers

| Layer | Tool | What it does |
|---|---|---|
| Read-only | `inspect_graph` | Nodes, topics, services and types |
| Read-only | `get_diagnostics` | Cached standard `/diagnostics` statuses |
| Read-only | `inspect_ros2_control` | Controllers, hardware components, interfaces |
| Read-only | `get_topic_snapshot` | One message from any discovered topic |
| Read-only | `sample_topic` | Time-window sampling with statistics |
| Read-only | `list_tests` | Deterministic test catalog |
| Read-only | `run_test` | Run one deterministic health test |
| Read-only | `query_history` | Recent test runs from SQLite |
| Read-only | `check_motion_safety` | Run the deterministic Safety Gate |
| **Emergency** | `emergency_stop` | Software stop: zero Twist + optional Trigger service. **Never blocked.** |

v0.1 contains no write or motion tool, so there is nothing to confirm. When
motion extensions land, the Safety Gate and an explicit operator confirmation
will gate them.

### Configuration

The agent resolves its LLM settings with this precedence:

```
CLI flags  >  environment variables  >  ~/.robodiag.env  >  built-in defaults
```

`~/.robodiag.env`:

```bash
DEEPSEEK_API_KEY=sk-...
DEEPSEEK_BASE_URL=https://api.deepseek.com
DEEPSEEK_MODEL=deepseek-chat
```

One-off overrides, including non-DeepSeek providers (any OpenAI-compatible endpoint):

```bash
./robodiag --model deepseek-reasoner
./robodiag --base-url https://dashscope.aliyuncs.com/compatible-mode/v1 --model qwen-max
./robodiag --api-key-file ~/.deepseek.key      # safer than --api-key
```

> `--api-key` works but is visible in `ps` and shell history. Prefer
> `DEEPSEEK_API_KEY` in `~/.robodiag.env` (mode `600`) or `--api-key-file`.

### Health thresholds

| Variable | Default | Meaning |
|---|---|---|
| `ROBODIAG_MIN_BATTERY_PCT` | `0.10` | minimum battery percentage for motion |
| `ROBODIAG_MIN_BATTERY_V` | `0.0` | minimum battery voltage for motion |
| `ROBODIAG_MAX_DIAG_AGE_S` | `5.0` | max age of `/diagnostics` before it is stale |
| `ROBODIAG_MAX_JOINT_STATE_AGE_S` | `1.0` | max age of `/joint_states` before it is stale |
| `ROBODIAG_DB` | `~/.robodiag_ros2.db` | SQLite history path |

---

## Test catalog

| `test_id` | Checks | Outcomes |
|---|---|---|
| `graph_health` | Node/topic counts; presence of `/joint_states` | `FAIL` if only the harness node is visible; `WARN` if `/joint_states` is missing |
| `diagnostics_health` | `/diagnostics` freshness and levels | `SKIP` if none received; `FAIL` on stale or ERROR/STALE; `WARN` on WARN |
| `joint_states_health` | Samples `/joint_states`: rate, finite values, named joints | `WARN` on low rate / few samples / missing positions |
| `ros2_control_health` | `controller_manager` controllers, hardware, interfaces | `SKIP` if unavailable; `FAIL` if hardware is not active |
| `system_health` | Composite of the four tests above | Worst sub-result |

Every run is written to SQLite and can be inspected with `/history` or the
`query_history` tool.

---

## Design principles

1. **Evidence before reasoning.** The agent must gather real telemetry; it may not
   invent hardware state from the robot model.
2. **Deterministic code decides writes.** The Safety Gate is plain code, is
   fail-closed, and distinguishes *unhealthy* evidence from *missing* evidence.
   "No data" is never treated as "normal".
3. **Structured results.** Evidence and tests are first-class objects
   (`Evidence`, `TestResult`), not free-form strings.

---

## Contributing

Issues and pull requests are welcome. The v0.1 catalog is intentionally
read-only; new tests should follow the same rule — deterministic, structured
evidence, and no motion without a Safety Gate precondition.

---

## License

Apache License 2.0 — see [LICENSE](LICENSE).

ROS and ROS 2 are trademarks of Open Robotics. This project is not affiliated
with or endorsed by Open Robotics or Intrinsic.

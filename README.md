# RoboDiag Harness 🩺🤖

**A command-line tool for checking your robot's health and investigating problems.**

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)
![Python](https://img.shields.io/badge/Python-3.10-blue)

![RoboDiag Harness terminal](assets/robodiag.jpg)

Your robot is connected—but is it working properly? Is sensor data arriving?
Are there any reported faults? What should you check when something goes wrong?

RoboDiag brings robot checks, telemetry, and diagnostic tools into one terminal
interface.

Think of it as a robot checkup: **collect evidence, inspect problems, and decide
what to investigate next.**

## What can I use it for?

- **Check your robot** — inspect its status and available diagnostic information.
- **Inspect sensor readings** — view telemetry and collect samples over time.
- **Investigate problems** — examine reported faults and review previous results.
- **Run supported tests** — use the checks and actions available for your robot.
- **Give an AI assistant access to diagnostic tools** — let it work with robot
  data through the harness.

Available features depend on the robot, its software, and the selected
configuration.

## Who is it for?

Robot owners, students, and developers who want a more convenient way to inspect
and troubleshoot their robots.

You will need basic familiarity with a terminal. For the ROS 2 version, you also
need a working ROS 2 environment.

You do **not** need an AI assistant to use the command-line interface.

## What does “Harness” mean?

A harness connects tools together so they can be used through a common interface.

Here, it connects robot data, diagnostic checks, and supported actions—so you
can access them from one place.

## Getting started with ROS 2

### 1. Prepare your robot

For most robots, the simplest setup is to run RoboDiag **on the robot's own
computer**, next to the robot's ROS 2 software:

- ROS 2 is installed on that computer.
- Your robot's ROS 2 software is running.
- RoboDiag can reach the robot's topics, services and actions over the local
  ROS graph.

You can also run RoboDiag **on a separate computer** (ROS 2 is distributed, so
nodes may live on different machines). In that case, additionally make sure
that:

- Both machines are on the same network and DDS discovery traffic is not
  blocked by a firewall.
- `ROS_DOMAIN_ID` matches on both machines.
- The ROS 2 distribution and message packages match — if the robot uses custom
  messages, source its workspace overlay (with `ROBODIAG_WS`, see step 3).

The launcher looks for ROS 2 under `/opt/ros` on the local machine. It uses the
distribution identified by `ROS_DISTRO` when available, otherwise it tries
Humble. RoboDiag does not use the robot's ROS installation remotely — the
machine it runs on must have ROS 2 installed.

### 2. Download the project

```bash
git clone https://github.com/YueBit/robodiag-harness.git
cd robodiag-harness
```

### 3. Start RoboDiag

```bash
bash ./robodiag
```

If your robot needs a workspace overlay, specify its location:

```bash
ROBODIAG_WS=~/robot_ws bash ./robodiag
```

The workspace should already be built, with an `install/setup.bash` file.

### 4. Start with a check

Begin with status checks and telemetry inspection. Review the available data
before acting on a conclusion.

Missing data does not necessarily mean broken hardware: the relevant sensor,
driver, or ROS 2 node may not be running.

## Configure your robot's interfaces

Robots may use different topic and service names. You can pass these to the
launcher:

```bash
bash ./robodiag \
  --cmd-vel-topic /cmd_vel \
  --estop-service /emergency_stop
```

Use the names provided by your robot's software. Specifying a name does not
create the corresponding topic or service.

## Using it with an AI assistant

An AI assistant can use diagnostic tools to collect information and help
interpret it.

A typical workflow is:

1. You describe the problem.
2. The assistant requests relevant robot data.
3. It reviews the results and suggests further checks.
4. You decide whether to act on its suggestions.

**AI suggestions can be wrong.** Check the underlying readings before acting on
a conclusion.

## Limitations

RoboDiag helps gather and interpret diagnostic evidence. It cannot guarantee
that a robot is safe or identify every hardware fault.

The checks it can perform depend on the information and control interfaces your
robot exposes. Support for ROS 2 does not mean every ROS 2 robot works without
configuration.

---

## Features

- Inspect the ROS graph: nodes, topics, services and their types.
- Collect standard `/diagnostics` (`diagnostic_msgs/DiagnosticArray`).
- Inspect `ros2_control` through `controller_manager` services, when available.
- Snapshot/sample **any** discovered ROS 2 topic by loading its message type at
  runtime; nothing is hard-coded.
- Time-window sampling with `min/max/mean/std/range` statistics for jitter,
  drift, dropouts and intermittency.
- Deterministic health tests (`PASS / WARN / FAIL / SKIP`) persisted to SQLite.
- A fail-closed Safety Gate for motion-related operations.
- A software emergency stop (zero Twist + optional Trigger service).
- An OpenAI-compatible diagnostic agent (DeepSeek by default) with function
  calling.
- A Jev (System One) next-tool router: Jev decides which read-only evidence
  tool to run next, then the LLM writes the final diagnosis.
- `rich` terminal UI with an animated banner and `/`-command completion.

## How it works

```
      natural language                         slash commands
            │                                        │
            ▼                                        ▼
   ┌────────────────────────────┐          ┌──────────────────┐
   │  Jev request router        │          │   REPL (rich)    │
   │  QUERY / DIAGNOSIS /       │          │  direct dispatch │
   │  ACTION / CHAT             │          └────────┬─────────┘
   └──────┬──────────┬──────────┘                   │
     QUERY│  DIAGNOSIS  │ACTION                     │
     concise│  Jev next-  │deterministic            │
     answer │  tool loop  │policy/safety            │
            │      │      │                         │
            │      ▼      │        same deterministic tools
            │  ┌─────────────────┐                    │
            │  │ LLM explainer   │                    │
            │  │ symptom +       │                    │
            │  │ evidence →      │                    │
            │  │ diagnosis       │                    │
            │  └────────┬────────┘                    │
            │           │                             │
            ▼           ▼                             ▼
   ┌──────────────────────────────────────────────────────────┐
   │                 Deterministic tool layer                 │
   │   HarnessNode (rclpy)  ·  TestRunner  ·  HistoryStore     │
   └───────────┬───────────────────────┬──────────────────────┘
               │                       │
       ROS 2 graph / topics       SQLite test history
               │
       ┌───────┴────────┐
       │  Safety Gate   │  fail-closed, no model in the loop
       │  diagnostics / │  checks freshness + battery + joints
       │  battery /     │
       │  joint states  │
       └────────────────┘
```

When `TYPESAFE_API_KEY` is not set, the classic LLM function-calling loop is
used instead of the Jev routing. The division of labour with Jev enabled:

> - **Jev request router classifies the task** (System 1 — QUERY/DIAGNOSIS/ACTION/CHAT).
> - **Jev diagnostic router decides what to investigate** (System 1, inside DIAGNOSIS).
> - **Deterministic tools decide what is true** (measured evidence only).
> - **The LLM explains what it means** (System 2 — written diagnosis).
> - **The Safety Gate decides what is allowed** (fail-closed, deterministic code).

The core safety rule is unchanged:

> **The LLM may request evidence and tests, but deterministic code decides
> whether a write or motion operation is permitted.**

### How is this different from `ros2 doctor`?

`ros2 doctor` inspects the local ROS installation and its configuration. RoboDiag
Harness inspects the **live running graph and robot telemetry**, samples topics
over time, runs deterministic health tests, stores history, and adds an LLM agent
that can decide *which* evidence to collect next.

## Project structure

The harness is split into three top-level Python modules plus a launcher, so the
safety-critical and decision-making pieces stay testable without a running robot.

| File | What it contains | ROS needed? |
|---|---|---|
| `robodiag_ros2.py` | Main CLI: the `rclpy` `HarnessNode`, the `rich` REPL, the LLM explainer, and the wiring that connects the tools, the Jev routers and the Safety Gate. | Yes |
| `robodiag_core.py` | Deterministic core (standard library only): JSON/statistics helpers, the SQLite `HistoryStore`, the read-only `TEST_CATALOG` and `TestRunner`, the fail-closed `SafetyGate`, and the ACTION capability registry. | No |
| `robodiag_jev.py` | Jev (System One) decision layer (standard library only): the `RequestRouter`, the `DiagnosticState` model, the TypeSafe API client, and the `JevRouter` that picks the next evidence tool. | No |
| `robodiag` | Bash launcher that sources ROS 2 and your robot workspace, then starts the Python harness. | — |

`robodiag_core.py` and `robodiag_jev.py` import only the Python standard library,
so the deterministic safety logic and the routing decisions can be unit-tested in
plain CI without ROS 2, `rich`, or network access.

## Install with pip

`pip install` provides the `robodiag` console command. **rclpy and the ROS
message packages are not on PyPI** — they come from your ROS 2 distribution, so
source ROS before running.

```bash
# from PyPI (once published)
pip install robodiag-harness             # core (rich only)
pip install "robodiag-harness[all]"      # + prompt_toolkit completion + openai agent

# from source / editable
pip install -e ".[all]"
```

> ROS 2 Humble images ship setuptools < 61 and may have no PyPI access. In that
> case build with the system setuptools instead of an isolated one:
>
> ```bash
> pip install --user --no-build-isolation ".[all]"
> ```

### Dependencies

- ROS 2 (tested on Humble; mostly distro-agnostic), Python 3.10
- `rich` (required)
- `prompt_toolkit` (optional, live completion)
- `openai` (optional, AI agent)
- `ros-humble-controller-manager-msgs` (optional, for ros2_control checks)

## REPL commands

![RoboDiag REPL commands](assets/repl-commands.jpg)

```
/graph                         ROS graph summary
/diagnostics                   Standard /diagnostics status
/control                       ros2_control status
/check                         Composite read-only health check
/topic <topic>                 Snapshot one message from a topic
/sample <topic> [sec] [Hz]     Sample a topic over time and compute statistics
/tests                         Test catalog
/run <test_id>                 Run a deterministic test (all read-only in v0.2)
/history [n]                   Recent n test runs
/safety                        Show the motion Safety Gate
/stop                          Software stop: zero cmd_vel + optional Trigger service
/help /quit
```

Any other input is classified by the Jev request router into `QUERY`,
`DIAGNOSIS`, `ACTION` or `CHAT` (requires `TYPESAFE_API_KEY`; `DIAGNOSIS` also
requires `DEEPSEEK_API_KEY` for the final explanation).

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
| `--jev-api-key` | *(none)* | TypeSafe API key (visible in `ps`/shell history) |
| `--jev-api-key-file` | *(none)* | read the TypeSafe API key from a file (safer) |
| `--jev-base-url` | `https://api.typesafe.ai` | TypeSafe base URL |
| `--jev-model` | `jev-latest` | Jev model; overrides `TYPESAFE_MODEL` |

## AI diagnostic agent

The agent runs a function-calling loop of at most 10 rounds. Each tool result is
fed back as structured JSON, and the final answer follows a fixed shape:
**symptom → evidence → assessment → possible causes → recommended next checks**.

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

v0.2 contains no write or motion tool, so there is nothing to confirm. When
motion extensions land, the Safety Gate and an explicit operator confirmation
will gate them. See [Motion extensions (future)](#motion-extensions-future)
for the plan.

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

TYPESAFE_API_KEY=apikey_...
TYPESAFE_BASE_URL=https://api.typesafe.ai
TYPESAFE_MODEL=jev-latest
```

One-off overrides, including non-DeepSeek providers (any OpenAI-compatible
endpoint):

```bash
./robodiag --model deepseek-reasoner
./robodiag --base-url https://dashscope.aliyuncs.com/compatible-mode/v1 --model qwen-max
./robodiag --api-key-file ~/.deepseek.key      # safer than --api-key
```

> `--api-key` works but is visible in `ps` and shell history. Prefer
> `DEEPSEEK_API_KEY` in `~/.robodiag.env` (mode `600`) or `--api-key-file`.
>
> Jev uses the same precedence: `--jev-*` flags > `TYPESAFE_*` environment
> variables > `~/.robodiag.env` > built-in defaults.

## Jev System One routing

With `TYPESAFE_API_KEY` set, Jev — TypeSafe's System One model — has two
separate routing roles:

1. **Request routing** — first classify the operator's input into one of
   `QUERY`, `DIAGNOSIS`, `ACTION` or `CHAT` (and, for `QUERY`/`ACTION`, resolve
   the specific capability/action).
2. **Diagnostic routing** — if (and only if) the input is a `DIAGNOSIS`, decide
   which evidence/tool to collect next.

```text
user input → Jev request router
               ├─ QUERY     → one deterministic lookup → concise answer
               ├─ DIAGNOSIS → Jev diagnostic loop → LLM explanation
               ├─ ACTION    → deterministic read-only test / software stop
               └─ CHAT      → lightweight reply (no tools)
```

Simple questions such as "what is the battery level?" are answered directly
(`Battery: 73% (15.6 V)`) without entering the diagnostic loop.

RoboDiag replies in the same language as the operator. The language is detected
from the script of the input (e.g. `现在电量多少？` → Chinese); when the language
is unclear (for example Latin-script text), replies default to English.

### Diagnostic routing

Inside a `DIAGNOSIS`, Jev acts as a fast **next-tool router**:

1. Jev receives the symptom and the evidence collected so far.
2. Jev picks the next read-only diagnostic action (a `Choice` over the tool set).
3. The deterministic tool runs and its result is appended to the evidence.
4. When Jev picks `finalize`, the LLM (System 2) writes the final diagnosis:
   symptom → evidence → assessment → possible causes → recommended next checks.

```text
symptom → Jev: get_diagnostics → Jev: run_test(joint_states_health)
        → Jev: sample_topic(/joint_states) → Jev: finalize → LLM report
```

Jev returns calibrated probabilities for every choice; the router follows the
highest-probability action and prints the top three with confidence. Jev may
*propose* the next tool and finalize, but deterministic gates decide what is
allowed:

- **Finalize gate** — `finalize` is withheld until at least one tool call has
  returned successful evidence (`MIN_EVIDENCE_ITEMS`).
- **Exact repeat guard** — the same `(tool, canonical arguments)` call cannot
  repeat; the same tool with *different* arguments (e.g. sampling two topics)
  is still allowed.
- **Per-tool budget** — each tool is capped at 3 calls per diagnosis.
- **Total bound** — at most 8 tool calls per diagnosis.
- **Topic shortlist** — topics are deterministically ranked (name tokens,
  message type, symptom keywords, prior evidence) down to ≤ 24, and Jev is
  always offered `none_of_the_above` so it is never forced to pick a bad topic.

**Jev is only allowed to route read-only evidence tools.** `emergency_stop` and
any future motion/write tool are never in Jev's action set — they stay behind
the deterministic Safety Gate and the `/stop` command.

The routing layer consumes a single `DiagnosticState` (`symptom`, `tool_calls`,
`test_results`, `evidence`, `graph_summary`, `step`) and returns a decision, so
Jev can later be swapped for an LLM-, rule-, or replay-based router without the
harness below changing.

| | Jev (System 1) | LLM (System 2) |
|---|---|---|
| Role | Decide what to investigate next | Explain what the evidence means |
| Output | Typed choice + calibrated probabilities | Free-form written diagnosis |
| Speed | ~70–500 ms, parallel, no generation | Seconds, autoregressive |
| Failure mode | Cannot hallucinate a tool call | Never executes tools; only writes |

With only `DEEPSEEK_API_KEY` (no Jev), the classic LLM function-calling agent
is used as the fallback. The `/jev` command shows which router is active.

### Health thresholds

| Variable | Default | Meaning |
|---|---|---|
| `ROBODIAG_MIN_BATTERY_PCT` | `0.10` | minimum battery percentage for motion |
| `ROBODIAG_MIN_BATTERY_V` | `0.0` | minimum battery voltage for motion |
| `ROBODIAG_MAX_DIAG_AGE_S` | `5.0` | max age of `/diagnostics` before it is stale |
| `ROBODIAG_MAX_BATTERY_AGE_S` | `5.0` | max age of battery state before it is stale |
| `ROBODIAG_MAX_JOINT_STATE_AGE_S` | `1.0` | max age of `/joint_states` before it is stale |
| `ROBODIAG_DB` | `~/.robodiag_ros2.db` | SQLite history path |

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

## Design principles

1. **Evidence before reasoning.** The agent must gather real telemetry; it may
   not invent hardware state from the robot model.
2. **Deterministic code decides writes.** The Safety Gate is plain code, is
   fail-closed, and distinguishes *unhealthy* evidence from *missing* evidence.
   "No data" is never treated as "normal".
3. **Structured results.** Evidence and tests are first-class objects
   (`Evidence`, `TestResult`), not free-form strings.
4. **Narrow decisions, composed in code.** Jev (System 1) only answers "which
   tool next?" with calibrated probabilities; the LLM (System 2) only writes
   the explanation. Neither model may choose an emergency stop or a
   motion/write action.

## Testing

The test suite is split into offline and live parts. The offline parts run
without ROS 2, `rich`, or any API key — they import only `robodiag_core` and
`robodiag_jev`.

```bash
python3 -m unittest test_core            # deterministic core (fully offline)
python3 test_jev.py                      # Jev next-tool router (offline + live)
python3 test_request_router.py           # request router (offline + live)
```

- **`test_core.py`** — unit tests for the deterministic core: JSON/statistics
  helpers, the SQLite history store, test-result semantics, the Safety Gate, and
  the routing whitelists. Runs fully offline and also works with
  `pytest test_core.py`.
- **`test_jev.py`** — smoke test for the Jev next-tool router. It always runs
  offline wiring checks; live TypeSafe API checks run only when
  `TYPESAFE_API_KEY` is available (environment variable or `~/.robodiag.env`)
  and are skipped otherwise.
- **`test_request_router.py`** — smoke test for the QUERY / DIAGNOSIS / ACTION /
  CHAT request router and language detection. Offline wiring checks always run;
  live classification runs only when `TYPESAFE_API_KEY` is available.

## Motion extensions (future)

v0.2 provides read-only diagnostic tests and a software stop command. It does
not initiate motion.

Future versions may add motion tests and other write actions. When they land:

- The Safety Gate and an explicit operator confirmation will gate them.
- Place the robot in a clear, stable area and keep people and obstacles away
  from moving parts before any movement test.
- Know how to stop your robot independently of RoboDiag.
- Review the requested action before confirming it.

A software stop command depends on the robot's implementation and communication
connection. It does **not** replace a physical emergency stop.

## Contributing

Bug reports, documentation improvements, and robot integrations are welcome.

When reporting an issue, please include:

- Your robot model and ROS 2 distribution.
- The command or check you ran.
- What you expected and what happened.
- Relevant logs, with credentials and private information removed.

The v0.2 test catalog is intentionally read-only; new tests should follow the
same rule — deterministic, structured evidence, and no motion without a Safety
Gate precondition.

## License

Apache License 2.0 — see [LICENSE](LICENSE).

ROS and ROS 2 are trademarks of Open Robotics. This project is not affiliated
with or endorsed by Open Robotics or Intrinsic.

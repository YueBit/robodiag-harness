# RoboDiag ROS 2 Diagnostic Harness

A terminal diagnostic harness for ROS 2 robots. It inspects the live ROS graph,
collects standard `/diagnostics`, dynamically snapshots/samples arbitrary topics,
runs deterministic read-only health tests, persists history to SQLite, and can
use an OpenAI-compatible LLM (e.g. DeepSeek) as an interactive diagnostic agent
with function calling.

> ⚠️ **Safety.** The included emergency-stop is a **software-level fallback** —
> it publishes a zero `geometry_msgs/Twist` and can optionally call a
> `std_srvs/Trigger` service. It is **not** a certified hardware E-stop.
> v0.1 ships no test that intentionally moves the robot. Deterministic code —
> never the LLM — decides whether a write/motion operation is permitted.

## Features

- Inspect the ROS graph: nodes / topics / services and their types
- Collect standard `/diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- Inspect `ros2_control` via `controller_manager` services when available
- Dynamically snapshot/sample arbitrary ROS topics without hard-coding message types
- Deterministic health tests with results persisted to SQLite
- Safety Gate for motion-related extensions
- Software emergency stop (zero Twist + optional Trigger service)
- Optional DeepSeek/OpenAI-compatible diagnostic Agent with function calling

## Requirements

- ROS 2 (tested on Humble; mostly distro-agnostic), Python 3.10
- `rich`: `python3 -m pip install --user rich`
- optional `openai` for the AI Agent: `python3 -m pip install --user openai`
- for `ros2_control` checks: `ros-humble-controller-manager-msgs`

## Quick start

```bash
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash   # your robot workspace, if any
python3 robodiag_ros2.py
```

Or use the launcher (auto-sources ROS, optional `ROBODIAG_WS` overlay):

```bash
./robodiag
./robodiag --cmd-vel-topic /cmd_vel
ROBODIAG_WS=~/robot_ws ./robodiag --estop-service /emergency_stop
```

## REPL commands

```
/graph                         ROS graph summary
/diagnostics                   Standard /diagnostics status
/control                       ros2_control status
/check                         Composite read-only health check
/topic <topic>                 Snapshot one message from a topic
/sample <topic> [sec] [Hz]     Sample over a time window and compute min/max/mean/std/range
/tests                         Test catalog
/run <test_id>                 Run a deterministic test (all read-only in v0.1)
/history [n]                   Recent n test runs
/safety                        Show the motion Safety Gate
/stop                          Software stop: zero cmd_vel + optional Trigger service
/help /quit
```

Any other natural-language input goes to the AI diagnostic Agent (requires
`DEEPSEEK_API_KEY`).

## Configuration

### AI Agent

The agent reads `~/.robodiag.env`, then falls back to environment variables.
Environment variables take precedence.

```
DEEPSEEK_API_KEY=sk-...
DEEPSEEK_BASE_URL=https://api.deepseek.com
DEEPSEEK_MODEL=deepseek-chat
```

### Health thresholds

| Variable | Default | Meaning |
|---|---|---|
| `ROBODIAG_MIN_BATTERY_PCT` | `0.10` | minimum battery percentage for motion |
| `ROBODIAG_MIN_BATTERY_V` | `0.0` | minimum battery voltage for motion |
| `ROBODIAG_MAX_DIAG_AGE_S` | `5.0` | max age of `/diagnostics` before it is stale |
| `ROBODIAG_MAX_JOINT_STATE_AGE_S` | `1.0` | max age of `/joint_states` before it is stale |
| `ROBODIAG_DB` | `~/.robodiag_ros2.db` | SQLite history path |

## Design rule

> The LLM may request evidence and tests, but deterministic code decides whether
> a write/motion operation is permitted.

Evidence and tests are structured: `graph → evidence → test → diagnosis`.
The default v0.1 catalog is entirely read-only.

## License

Apache License 2.0 — see [LICENSE](LICENSE).

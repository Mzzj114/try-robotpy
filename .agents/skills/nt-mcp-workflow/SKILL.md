---
name: nt-mcp-workflow
description: Use when investigating an FRC robot issue with the nt-mcp-server NetworkTables MCP server.
license: MIT
metadata:
  author: Randy Z
  version: "1.0.0"
---

# nt-mcp-workflow

Investigate an FRC robot issue using the `nt` (NetworkTables) MCP server. The server speaks NT4 and works both **live** (connected to a running RobotPy sim or real robot) and **offline** (replaying a saved `.ndjson` recording).

## When to use this skill

- You need to read, write, or monitor NetworkTables values to diagnose robot behavior.
- You have a saved NT recording and need to inspect what happened after the fact.
- The user asks you to tune, debug, or analyze a robot subsystem through telemetry.

## Choose the investigation mode

| Mode | Best for | See |
| --- | --- | --- |
| **Live online** | A focused, interactive issue (wrong value, single subsystem, real-time inputs). | `references/automated-workflow.md` |
| **Offline recording** | A broad or transient issue (whole-match forensics, post-event analysis, unreproducible glitch). | `references/replay-recording.md` |
| **Capture telemetry** | You need a recording of what happened — to feed offline analysis or to keep a capture window out of your context. | `references/recording.md` |

If neither mode is a perfect fit, use whichever tools best answer the user's question.

## Capture telemetry

Offline analysis needs a recording. Two ways to produce one, both writing the same NDJSON format:

- `nt-recorder` CLI — a standalone process that records until stopped; best for long captures, background recording, or capturing before the agent connects. See `references/recording.md` for flags, detached startup on Windows, and shutdown.
- `nt_subscribe(output="file")` — the MCP tool captures a bounded window and returns a compact receipt (`recording_id`, `path`, `rows`, `topic_count`, ...) with no sample values; window into the recording afterwards with `nt_get_history` / `nt_subscribe_offline`. Also covered in `references/recording.md`.

Before drawing conclusions from any recording, read `references/analysis-gotchas.md` — the disabled-window, sampling-aliasing, and truncation traps are documented there and have caused real misdiagnoses.

Connecting live to a **real robot** (rather than the sim) requires a two-adapter network setup with static routing — see `references/network-setup.md`.


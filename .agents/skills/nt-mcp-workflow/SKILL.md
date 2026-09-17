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

If neither mode is a perfect fit, use whichever tools best answer the user's question.

Connecting live to a **real robot** (rather than the sim) requires a two-adapter network setup with static routing — see `references/network-setup.md`.


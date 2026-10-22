# Recording NetworkTables

Use `nt-recorder` to capture live NetworkTables data to NDJSON files for offline analysis. The recorder is a standalone CLI, independent of the MCP server. You do not need the MCP server running to record.

## CLI usage

```powershell
uv run nt-recorder --prefixes /SmartDashboard/ --output-dir recordings
```

### Flags

| Flag | Default | Description |
| --- | --- | --- |
| `--prefixes` | `/` | Topic prefixes to subscribe to. Pass multiple: `--prefixes /SmartDashboard/ /FMSInfo/` |
| `--output-dir` | `./recordings` | Directory for output files. Created automatically if it does not exist. |
| `--duration` | None (run forever) | Record for N seconds, then exit cleanly. |
| `--team` | None | Connect via FRC team number instead of server IP. Mutually exclusive with a non-default `--server-ip`. |
| `--server-ip` | `127.0.0.1` | NT4 server address. |
| `--server-port` | `5810` | NT4 server port. |
| `--identity` | `nt-recorder` | Client identity string reported to the NT4 server. |
| `--quiet` | off | Suppress status output to stderr. |

If both `--team` and a non-default `--server-ip` are passed, the recorder exits with code 1 and prints an error to stderr. The two targeting methods are mutually exclusive.

### Output format

Each line in the `.ndjson` file is a JSON object:

```json
{"time": 1.234, "topic": "/SmartDashboard/Pose X", "value": 1.5}
```

Three fields, always present: `time` (float, NT4 timestamp seconds), `topic` (string), `value` (JSON-serializable).

### Filename scheme

Files are named `nt-record-<YYYY-MM-DDTHHMMSSZ>.ndjson`. The timestamp is UTC, colons are stripped so the name is Windows-safe.

### Exit codes

| Code | Meaning |
| --- | --- |
| 0 | Clean exit (duration elapsed, or Ctrl+C / SIGINT) |
| 1 | Connection failure (could not reach the NT4 server within the timeout) |
| 2 | Disk or I/O error (write failed, directory not writable, etc.) |

### Environment variable

`NT_RECORDINGS_DIR` sets the default output directory for both the CLI and the offline MCP tools. When unset, both default to `./recordings`. The `--output-dir` flag overrides this for the CLI.

## Running detached on Windows

To record in the background without blocking the agent turn, start the recorder as a hidden process:

```powershell
Start-Process -FilePath ".venv\Scripts\python.exe" `
  -ArgumentList "-m", "nt_mcp_server.nt_recorder", "--prefixes", "/SmartDashboard/", "--output-dir", "recordings" `
  -PassThru -WindowStyle Hidden `
  -RedirectStandardOutput "recordings\nt_recorder.out" `
  -RedirectStandardError  "recordings\nt_recorder.err"
```

The `-PassThru` flag returns a process object you can capture for later reference.

## Stopping a detached recorder

Match the command line to find the right process, then kill it:

```powershell
Get-CimInstance Win32_Process -Filter "Name='python.exe'" |
  Where-Object { $_.CommandLine -like '*nt-recorder*' } |
  ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
```

This finds any `python.exe` whose command line contains `nt-recorder` and force-stops it. The recorder flushes each NDJSON line as it is written, so partial files are still valid up to the last flushed line.

## Stop the recorder between test sessions

A live recorder left running keeps growing the output file and mixes data from different test runs (enabled windows, disabled windows, different tuning configurations). Always stop the recorder before starting a new session. If you are using the detached-on-Windows pattern, run the stop command above before launching a new recorder.

## The newer path: live capture via `nt_subscribe`

The `nt_subscribe` MCP tool's default `output="file"` mode produces the same NDJSON format as `nt-recorder`. Instead of running a separate CLI process, the agent calls the tool, which captures the window into a recording file and returns a compact receipt:

```json
{
  "connected": true,
  "output": "file",
  "recording_id": "nt-record-2026-09-15T143022Z.ndjson",
  "path": "/path/to/recordings/nt-record-2026-09-15T143022Z.ndjson",
  "duration_seconds": 10.0,
  "rows": 847,
  "topic_count": 12,
  "topics": ["/SmartDashboard/Pose X", "..."],
  "topics_truncated": false,
  "truncated": false
}
```

The receipt contains no sample values. To inspect the captured data, window into it with the offline tools (`nt_get_history`, `nt_subscribe_offline`). This keeps the agent context small: capture is a file write, analysis is a separate read.

Both paths (CLI recorder and tool-based capture) write the same three-column NDJSON, so an offline workflow works identically regardless of how the recording was produced.

## Offline analysis

Once you have a recording, use the offline MCP tools to inspect it:

1. `nt_list_recordings` to find available files.
2. `nt_get_recording_info(recording_id=...)` for a quick overview (duration, sample count, topic count).
3. `nt_list_topics_offline(recording_id=...)` to discover which topics are present.
4. `nt_subscribe_offline(prefixes=[...], recording_id=...)` or `nt_get_history(topic=..., recording_id=...)` to pull samples.

See `references/replay-recording.md` for the full offline workflow.

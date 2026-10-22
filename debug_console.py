"""NetworkTables debug console for live Python evaluation on the robot.

WARNING: this exposes arbitrary remote code execution (``eval``/``exec``) to
anyone who can write to the ``/Debug/Cmd`` NetworkTables topic. It exists only
to aid on-robot debugging and experimentation. It must NOT be used during a
competition match.

Protocol
--------
- ``/Debug/Cmd`` (string): writing a new value runs it once, in the robot loop,
  right after the command scheduler runs.
- ``/Debug/Out`` (string): the result of the last command, or the traceback of
  its failure. Captured ``print()`` output is included above the result.

The evaluation namespace persists across commands, so a name defined in one
command is visible to the next (REPL-style). Raw expressions (``robot.swerve
.getPose()``) are evaluated and their ``repr`` returned; statements
(``robot.swerve.stop()``) are executed.

NetworkTables suppresses a write whose value is unchanged, so re-sending the
exact same string is silently ignored and does not run a second time. Change
the text (e.g. add a trailing space or a sequence number) to force a re-run.

The command runs on the robot's main loop thread, so a command that blocks
(sleeps, waits, spins) will stall the entire robot. Keep commands fast.
"""

from __future__ import annotations

import contextlib
import io
import traceback
import typing

import commands2
import ntcore
import wpilib

# Cap the published result so a huge repr cannot overload NetworkTables.
_MAX_RESULT_CHARS = 4096


class DebugConsole:
    """Polls a NetworkTables string and ``eval``/``exec``s it against the robot.

    :param robot: the live top-level robot instance, exposed as ``robot`` in
        the evaluation namespace.
    """

    def __init__(self, robot: object) -> None:
        table = ntcore.NetworkTableInstance.getDefault().getTable("/Debug")

        # NetworkTableEntry (rather than StringEntry) is used because only it
        # exposes getValue(), which carries the sample timestamp used to tell a
        # new command from the previously executed one.
        self._cmd_entry: ntcore.NetworkTableEntry = table.getEntry("Cmd")
        self._out_entry: ntcore.NetworkTableEntry = table.getEntry("Out")

        # Persistent namespace: shared by both eval and exec so state defined by
        # one command is available to later commands.
        self._namespace: dict[str, typing.Any] = {
            "robot": robot,
            "wpilib": wpilib,
            "commands2": commands2,
            "ntcore": ntcore,
        }

        # Timestamp of the last command we ran. NetworkTables suppresses a write
        # whose value is unchanged, so re-sending the exact same string is a
        # no-op; to run it again, send slightly different text (e.g. a trailing
        # space or a sequence number).
        self._last_cmd_time = -1

    def periodic(self) -> None:
        """Run a newly published command, if any. Call once per robot loop."""
        value = self._cmd_entry.getValue()
        if not value.isValid() or value.time() == self._last_cmd_time:
            return

        self._last_cmd_time = value.time()
        if not value.isString():
            return

        code = value.getString()
        if not code.strip():
            return

        self._run(code)

    def _run(self, code: str) -> None:
        """Compile, evaluate, and publish the result of ``code``."""
        # Compile as an expression first; statements (assignments, imports,
        # loops) fail that and are compiled as exec instead. Compiling up front
        # guarantees the code is not executed twice.
        try:
            compiled = compile(code, "<debug>", "eval")
            is_expression = True
        except SyntaxError:
            compiled = compile(code, "<debug>", "exec")
            is_expression = False

        stdout = io.StringIO()
        result: typing.Any = None
        error: str | None = None

        # Redirect stdout/stderr so print() inside the command is visible.
        with contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stdout):
            try:
                if is_expression:
                    result = eval(compiled, self._namespace)
                else:
                    exec(compiled, self._namespace)
            except BaseException:  # noqa: BLE001 - report everything to the user
                error = traceback.format_exc()

        self._out_entry.setString(self._format(stdout.getvalue(), result, error))

    @staticmethod
    def _format(printed: str, result: typing.Any, error: str | None) -> str:
        """Merge captured output, the result, and any traceback into one string."""
        parts: list[str] = []
        if printed:
            parts.append(printed.rstrip("\n"))
        if error is not None:
            parts.append(error)
        elif result is not None:
            parts.append(repr(result))

        text = "\n".join(parts) if parts else "OK"
        if len(text) > _MAX_RESULT_CHARS:
            text = text[:_MAX_RESULT_CHARS] + f"\n... [truncated at {_MAX_RESULT_CHARS} chars]"
        return text

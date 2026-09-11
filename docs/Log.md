# Log module

Two independent loggers with different jobs — don't confuse them when
reading call sites.

## `ProgramLogger` (`programLogger.h`/`.cpp`)

Human-readable text log — what `LOG_DEBUG`/`LOG_INFO`/`LOG_WARNING`/
`LOG_ERROR` (the macros most of the codebase actually calls) write to.
Singleton (`PROGRAM_LOGGER`/`instance()`), writes to a single file set via
`setLogFileName()` (called once in `main.cpp`), mutex-guarded so any thread
can log safely. `enableVerbose()` gates `LogLevel::debug` messages.

## `Logger` (`logger.h`/`.cpp`)

Structured, per-`LogType` CSV dump (`MPC_ARG_X0`, `STATES`, `CONTROLS`,
etc. — see `Definitions/logDefinitions.h`) — one file per `LogType`, opened
lazily. Backed by a `ThreadSafeQueue<LogItem>` (`threadSafeQueue.h`) and a
dedicated writer thread (`m_writerLoop()`), so callers on the control-loop
thread never block on file I/O — `log()` just pushes onto the queue.
`start(enabled, logDirectory)` is what actually turns logging on;
`enabled=false` makes every `log()` call a no-op, used to skip the heavy
per-tick CSV dump when `verboseLogging` isn't set in config.

**`LogType::NMPC_EVENT` is the one exception to the enabled/disabled
gate** — sparse, human-readable NMPC controller transition events (launch,
in-flight, trajectory loaded/ended, solver violation entered/cleared,
emitted by `NMPCController::m_logTransitions()`) are always written,
regardless of `verboseLogging`, since they're low-volume and useful even
without full telemetry logging on.

## `ThreadSafeQueue<T>` (`threadSafeQueue.h`)

Generic blocking queue (`push`/`pop`/`stop`/`clear`) — `Logger`'s only
consumer today, but has no `Logger`-specific dependencies.

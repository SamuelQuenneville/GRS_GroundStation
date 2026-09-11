# Console module

`ConsoleInterface` (`consoleInterface.h`/`.cpp`) is the operator's
command-line interface, run on its own thread (`m_listen()`, started by
`main.cpp` after `GroundControlStation::initialize()`). Reads lines from
stdin and dispatches them (`m_dispatch()`) to the corresponding
`GroundControlStation` method — `connect`, `arm`, `setMode`, `genTraj`,
catapult commands, `origin <lat> <lon> <alt>` (parsed by the static
`parseOrigin()`), etc. `printCommands()` (also reachable via `--listCommand`
on the CLI, see `docs/ARCHITECTURE.md`) lists every available command.

Holds a reference to the shared exit condition variable/mutex/flag set up
in `main.cpp`, so a console `exit`/`quit` command can signal the same clean
shutdown path the OS-level wait in `main()` blocks on.

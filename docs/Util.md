# Util module

Small, dependency-light helpers with no natural home elsewhere.

- **`parseUtils.h`** — `grs::parseInt<T>()`/`grs::parseDouble()`: no-throw
  wrappers around `std::from_chars` for turning user-typed text (CLI args,
  console commands) into numbers, returning `std::nullopt` on bad input
  instead of throwing — used by `main.cpp`'s CLI parsing and
  `ConsoleInterface`. `grs::trim()` is the shared whitespace-trimming
  helper both use internally.
- **`profilingTimer.h`** — `ProfilingTimer`, a scope-guard that measures
  wall time between construction and destruction, optionally writing the
  result to an output pointer and/or logging it. `PROFILE_SCOPE(name)` /
  `PROFILE_SCOPE_OUT(name, outptr, print)` macros wrap it for one-line use
  at a call site.

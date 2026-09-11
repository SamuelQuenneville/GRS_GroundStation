# Definitions module

Shared plain-data structs with no behavior of their own — the common
vocabulary that lets modules pass data across boundaries without depending
on each other's headers (see "Event propagation" in `docs/ARCHITECTURE.md`).

| File | Contents |
|---|---|
| `communicationStructures.h` | `uavStates` (packed, wire-compatible with the MATLAB UDP link — see the top-level `README.md`), `uavCommands`/`uavCommandsFlags`, `uavHealth`, `subscriptionHandles`, `aggregatorRates` |
| `controllerStructures.h` | `solverConfig` — dimensions and bounds the CasADi solver was built with (`nx`/`nu`/`np`/`N`/`numUavs`, state/control bounds and scales) |
| `catapultProtocol.h` | Wire protocol shared between the GCS and the ESP32 catapult firmware — see `docs/Communication.md` |
| `gpsDefinitions.h` | Struct layouts (`sensor_gps_s`, etc.) required by the vendored PX4-GPSDrivers library — see below |
| `logDefinitions.h` | `LogType`/`LogItem` — see `docs/Log.md` |
| `yamlCustomStructure.h` | `YAML::convert<T>` specializations that let `yaml-cpp` decode config structs directly — see `docs/Configuration.md` |

**`uavStates` is packed and wire-shared with MATLAB** (`__attribute__((packed))`,
field order matches the MATLAB struct documented in the top-level
`README.md`) — adding, removing, or reordering fields breaks that link.
Non-numeric/low-rate status belongs in `uavHealth` instead, which has no
such constraint.

**`gpsDefinitions.h` is a required interop header, not vendored code.**
PX4-GPSDrivers (used by `RtkBaseStation`, see `docs/Communication.md`)
expects the consuming project to supply a `definitions.h` with these exact
struct layouts, plus `GPS_INFO`/`GPS_WARN`/`GPS_ERR` macros and
`gps_absolute_time()` — normally provided by PX4-Autopilot or
QGroundControl. This file is the minimal equivalent for this project,
copied from PX4-Autopilot's own `drivers/gps/definitions.h`, and included
only via the `GPS_DEFINITIONS_HEADER` compile definition in `CMakeLists.txt`
(not included directly elsewhere). Treat it like vendored code for the
comment-reduction pass — its structure is dictated by the external library,
not by this project's own conventions.

# Communication module

Everything that talks to hardware over a network link: vehicles (MAVSDK/
MAVLink), the catapult launchers (custom TCP protocol), and the RTK base
station (serial, RTCM3). See `docs/ARCHITECTURE.md` for how this fits into
the rest of the system.

## `CommunicationManager` (`communicationManager.h`/`.cpp`)

Owns one `mavsdk::Mavsdk` instance and, per connected vehicle (keyed by
MAVLink sysId), a set of MAVSDK plugin instances (`Telemetry`, `Action`,
`Param`, `MavlinkPassthrough`, `Rtk`).

**Registration is fully event-driven, not connect-and-wait.**
`subscribe_on_new_system` (`m_watchSystem`) fires whenever MAVSDK sees a
system on any open connection, at any time — not just during a bounded
"connect" window — so vehicles can power on in any order, at any pace,
without racing MAVSDK's post-heartbeat handshake. `m_watchSystem` attaches
an `is_connected` watcher if a system isn't fully ready yet, and
`m_registerSystem` (idempotent) creates the plugin instances once it is.
`connectAll()`'s `discoveryTimeoutMs` only bounds how long the *caller*
blocks for an initial status summary — registration itself has no timeout
and keeps happening in the background after `connectAll()` returns.

**Two telemetry paths, deliberately separate:**
- Numeric, high-rate state (`setTelemetryCallback`) — position, velocity,
  attitude, airspeed — assembled per-UAV by a `StatesAggregator` (see
  below) and consumed by the control loop.
- Non-numeric, low-rate status (`setStatusCallback`) — health, battery, GPS
  fix, RC link, armed state, flight mode, connection — what the dashboard's
  Status/Health cards are built from. `uavHealth::customMode` is read
  directly off the raw HEARTBEAT message rather than through MAVSDK's own
  flight-mode translation, because the project's ArduPlane fork
  ("GrsPlane") has custom mode numbering that MAVSDK's stock-ArduPilot/PX4
  translation table doesn't know about (see `gcsConfig.h`'s
  `flightModeToString()` for the matching decode).

Vehicle commands (`setUavCommands()`) go out as `SET_ATTITUDE_TARGET`
messages built by `MavlinkMessageBuilder` (see below), sent over
`MavlinkPassthrough` at a fixed rate by `m_sendAttitudeTarget()`.

## `StatesAggregator` (`statesAggregator.h`/`.cpp`)

One instance per UAV. MAVSDK delivers attitude, position, velocity,
airspeed, and global-position as separate, independently-rated
subscriptions; `StatesAggregator` merges the latest value from each into
one `uavStates` snapshot (`getSnapshot()`) and tracks each subscription's
observed update rate (`getRates()`, used for diagnostics — e.g. spotting a
stalled GPS feed).

## `MavlinkMessageBuilder` (`mavlinkMessageBuilder.h`/`.cpp`)

Static helper that builds a `SET_ATTITUDE_TARGET` MAVLink message from a
`uavCommandsFlags` (roll/pitch/yaw/thrust), including the
Euler-to-quaternion conversion the message requires.

## `CatapultLauncher` (`catapultLauncher.h`/`.cpp`)

Controls N launch catapults (nominally 2), each an ESP32 that dials **in**
to the GCS over Wi-Fi/TCP — one listen port per catapult
(`CatapultEndpoint`). Wire protocol is `Definitions/catapultProtocol.h`
(shared verbatim with the ESP32 firmware).

**Firing simultaneity**: every catapult gets `FIRE_AT(countdownMs)`
back-to-back, then each board fires off its own local hardware timer after
the countdown elapses — this depends only on one-way network jitter between
the GCS and each board, not on round-trip time or the boards sharing a
clock.

**Safety model**: explicit two-step ARM → FIRE. `armAll()` requires every
catapult to ack "cocked + armed" before returning success; on partial
failure, whichever catapults did arm are automatically disarmed so a single
catapult is never left live. Each board runs its own GCS-heartbeat watchdog
and self-disarms if it loses contact; the GCS mirrors this by watching
heartbeats too and reports `Fault` if a link goes quiet while
armed/counting down. `fireAll()`'s `acceptTimeoutMs` is a real abort
window — if any catapult doesn't confirm receipt of `FIRE_AT` in time,
`ABORT` is sent to every catapult immediately, so `acceptTimeoutMs` must
stay comfortably shorter than `countdownMs`.

## `RtkBaseStation` (`rtkBaseStation.h`/`.cpp`)

Wraps a u-blox F9P RTK receiver (via the PX4-GPSDrivers `GPSDriverUBX`
library — see `docs/Geo.md`'s note on `Definitions/gpsDefinitions.h`) over a
serial connection (`RtkSerialPort`). `start()` kicks off the F9P's
self-survey (`surveyInMinimumMeters`/`surveyInDurationSeconds`), then
streams RTCM3 correction bytes to `callback` as they're produced.
`scanAvailablePorts()`/`findFirstMatchingPort()` auto-detect the F9P by USB
vendor ID (`/sys/class/tty`, Linux only) rather than requiring a hardcoded
device path.

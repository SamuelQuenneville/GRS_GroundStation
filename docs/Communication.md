# Communication module

Everything that talks to hardware over a network link: vehicles (MAVSDK/
MAVLink), the catapult launchers (custom TCP protocol), and the RTK base
station (serial, RTCM3). See `docs/ARCHITECTURE.md` for how this fits into
the rest of the system.

## `CommunicationManager` (`communicationManager.h`/`.cpp`)

Owns one `mavsdk::Mavsdk` instance and, per registered vehicle (keyed by
MAVLink sysId), a `Vehicle`: its `Telemetry`, `Param`, `MavlinkPassthrough`
and `Rtk` plugins and its subscription handles.

**Registration is event-driven.** `subscribe_on_new_system` calls
`m_watchSystem` whenever MAVSDK sees a system on any link, at any time, so
vehicles can power on in any order. `m_watchSystem` attaches one
`is_connected` watcher per sysId (and handles a system already connected
when first seen). On the first connection a registration thread creates the
plugins and subscribes (`m_registerSystem`, idempotent); on a later
reconnection (vehicle reboot) the message rates are requested again and the
vehicle is marked connected. `connectAll()` also watches the systems MAVSDK
already knows, so connect works again after `stop()`. `discoveryTimeoutMs`
only bounds how long `connectAll()` waits to print a summary.

**Threads.** MAVSDK callbacks, registration threads, the publish thread and
the callers (console, dashboard, dispatcher) share the maps: `m_linkMutex`
guards the vehicles and watchers, `m_statesMutex` the aggregators and
healths, `m_requestMutex` the request bookkeeping. Plugins are copied out
(`shared_ptr`) under the lock and used outside it. `stop()` moves everything
out under the lock and unsubscribes outside it (the callbacks take the same
locks), after joining the registration threads; every MAVSDK subscription,
raw MAVLink ones included, is released.

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
`MavlinkPassthrough` as soon as the control loop produces them.

**Link budget.** Every message is requested by the GCS with
`SET_MESSAGE_INTERVAL` on connection; set every `SRx_*` of the GCS port to 0
on the vehicle so nothing else is streamed:

| Message | Rate | Use |
|---|---|---|
| `CONTROL_SYSTEM_STATE` (146) | `stateRateHz` (50) | controller and NMHE state, GRS convention (`grsMavlinkConventions.h`) |
| `GLOBAL_POSITION_INT` | 5 Hz | GCS origin, frame-offset check, dashboard |
| `SYS_STATUS`, `GPS_RAW_INT`, `BATTERY_STATUS`, `RC_CHANNELS` | 1 Hz | dashboard |
| `HEARTBEAT` | 1 Hz (always) | mode, armed, link |
| `GPS_GLOBAL_ORIGIN` | on request (2 s until known, then 10 s) | frame offset |

`CONTROL_SYSTEM_STATE` is requested again (at most every 2 s, with a
warning) while it is missing or older than 1 s, so a vehicle without the GRS
firmware is reported and a rebooted one resumes. Its measured rate is logged
after the first 5 s and whenever it drops below 70 % of the request.

## `StatesAggregator` (`statesAggregator.h`/`.cpp`)

One instance per vehicle. Merges `CONTROL_SYSTEM_STATE` (position,
velocity, airspeed, attitude from the quaternion, all from one EKF sample)
and `GLOBAL_POSITION_INT` into one `uavStates` snapshot (`getSnapshot()`).
A state not newer than the last (`time_usec`) is dropped. `lastStateTime()`
gives the arrival time of the last state, the telemetry age used by the
control loop's staleness check.

## `MavlinkMessageBuilder` (`mavlinkMessageBuilder.h`/`.cpp`)

Static helper that builds a `SET_ATTITUDE_TARGET` MAVLink message from a
`uavCommandsFlags`, including the Euler-to-quaternion conversion the message
requires. Attitude goes in `q`, thrust in `thrust`; the GrsPlane fork reads
the body-rate fields as data (`type_mask` 0):

| Field | Content |
|---|---|
| `body_roll_rate` | angle-of-attack feedforward [deg], 0 if none |
| `body_pitch_rate` | tether tension [N], 0 if none |
| `body_yaw_rate` | `commandFlag` bits as a float: bit 0 should_move, bit 1 end_sim, bit 2 launch |

A command file (CSV mode) gives the same values per line: time, sysId, roll,
pitch, yaw, thrust, aoa, tension, flags.

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

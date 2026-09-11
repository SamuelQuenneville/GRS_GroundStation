# Geo module

`GeodeticConverter` (`geodeticConverter.h`/`.cpp`) converts between WGS84
geodetic coordinates (latitude/longitude/altitude) and a local NED frame,
via ECEF as the intermediate step (standard geodetic-to-ECEF-to-NED
pipeline, WGS84 ellipsoid constants at the top of the header).

`initializeReference(lat, lon, alt)` sets the origin everything else is
relative to; `geodeticToNed()`/`ecefToNed()` convert a point into that
frame. `isInitialized()` reports whether a reference has been set yet.
This is the low-level math `NavigationFrameManager` (see `docs/Control.md`)
builds on — `GeodeticConverter` itself has no notion of per-vehicle offsets
or of when it's safe to convert; that bookkeeping lives one layer up.

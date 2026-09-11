# Configuration module

Loads the GCS's YAML config file into the plain structs the rest of the
codebase uses (`gcsConfig`, `solverConfig`, `pixhawkConfig`,
`pixhawkEndpointConfig`, `catapultEndpointConfig` — most defined in
`source/gcsConfig.h`).

`ConfigurationParser::parseGcsConfig(node, defaults)` reads the
`GcsConfiguration`, `Pixhawk`, and `Catapults` top-level YAML keys into a
`gcsConfig`, starting from `defaults` so any key the YAML doesn't set keeps
its default value — `main.cpp` calls this once at startup, then applies CLI
overrides on top (see `docs/ARCHITECTURE.md`'s "Process shape").
`parseSolverConfig(node)` separately reads the `SolverConfiguration` key
into a `solverConfig` — `ControlInterface::initialize()` calls this only in
MPC mode, since MATLAB/ATTITUDE_FILE modes have no solver to configure.

The actual per-field decoding lives in `Definitions/yamlCustomStructure.h`'s
`YAML::convert<T>` specializations, which `yaml-cpp` invokes via `.as<T>()`
— `configurationParser.cpp` itself is just the two entry points above plus
picking which top-level YAML node to hand to which type.

# Mathematics module

Shared math primitives used throughout the codebase.

- **`mathUtils.h`** — `grs::degToRad<T>()`/`grs::radToDeg<T>()`, templated
  over any floating-point type.
- **`vector.h`** / **`matrix.h`** — small templated `Vector<T, N>`/
  `Matrix<T, N>` types (arithmetic operators, common operations) used for
  fixed-size 2D/3D/4D math.
- **`math.h`** — the header everything else actually includes; pulls in the
  two above and defines the convenience aliases (`Vec3d`, `Matrix3d`, etc.)
  used everywhere (`NavigationFrameManager`, `TrajectoryGenerator`, `Geo`).

Already carries real Doxygen-style comments (`@brief`/`@param`/`@return`) —
the only module in the codebase that does today. Treat this as the
reference style for the IDE-friendly Doxygen pass on the rest of the
codebase, rather than rewriting it.

# Modernization Notes

## Scope

This project is being modernized as repository archaeology and behavior preservation, not as a rewrite.

## Phase 0 findings (summary)

- The reverse patch is introduced in commit `f2ab006`.
- The patch adds `--reverse`/`-2`, `DirectionMode`, and shared transfer helpers.
- Build system is legacy autotools with committed generated files.
- No meaningful `make check` test suite existed.

## Current implementation direction

1. Keep fork identity explicit as iperf2-era historical code.
2. Add parameterized behavior tests (`tests/reverse-cases.tsv` + `tests/reverse-matrix.sh`).
3. Compare legacy behavior against stock `iperf3 -R`.
4. Point users to upstream `iperf3` for forward use when equivalence is validated.

- A C++ toolchain compatibility fix was added in `include/headers.h` to avoid `bool` macro conflicts from legacy generated config headers.
- Fixed `-Wformat-security` compiler warnings in `compat/Thread.c` and `src/` files (e.g., `ReportDefault.c`, `Reporter.c`, `Settings.cpp`).
- Hardened the `autotools` build system to pass `make distcheck`:
    - Included missing `Dataxfer.hpp` in the distribution.
    - Included `tests/` and `docs/` files in the distribution.
    - Updated `check-local` in `Makefile.am` for robust out-of-tree testing.

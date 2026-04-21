# Repository Map

This map outlines the logical structure of the `iperf` legacy fork.

## 1. Core Logic (`src/`)
Contains the primary implementation of the iperf engine.
- `main.cpp`: Entry point.
- `Client.cpp` / `Server.cpp`: High-level role execution.
- `Listener.cpp`: Connection acceptance logic.
- `Dataxfer.cpp`: **[PATCHED]** Core data transfer loops for TCP and UDP.
- `Settings.cpp`: **[PATCHED]** Command-line parsing and internal state management.
- `Reporter.c` / `ReportDefault.c`: Results reporting and formatting.
- `PerfSocket.cpp`: Socket initialization and option setting.

## 2. Public Interfaces (`include/`)
Header files for the core logic.
- `Settings.hpp`: **[PATCHED]** Defines `DirectionMode` and `thread_Settings`.
- `Dataxfer.hpp`: Interface for data transfer functions.
- `headers.h`: **[FIXED]** Global inclusion and compatibility macros.

## 3. Compatibility Layer (`compat/`)
Provides portable implementations of system functions for older or non-POSIX systems.
- `Thread.c`: **[FIXED]** Multi-threading abstraction.
- `gettimeofday.c`: Portable time measurement.
- `inet_ntop.c` / `inet_pton.c`: Portable address conversion.
- `delay.cpp`: Micro-delay implementation for UDP bandwidth limiting.

## 4. Build System & Metadata
- `configure.ac`: Autoconf script.
- `Makefile.am`: Automake templates (found in root, `src`, `include`, `compat`, `doc`, `man`).
- `m4/`: Custom autotools macros (e.g., `dast.m4`).
- `autogen.sh`: Script to regenerate the build system.

## 5. Documentation & Tests
- `doc/`: Legacy HTML documentation.
- `man/`: Iperf man page.
- `tests/`: **[NEW]** Automated behavior matrix and NAT simulation scripts.
- `docs/`: **[NEW]** Modernization notes, strategy, and archaeology maps.

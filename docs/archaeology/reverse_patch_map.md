# Reverse Patch Map

This document traces the implementation of the `--reverse` (`-2`) functionality in this legacy fork.

## 1. Data Structures (`include/Settings.hpp`)
- **`DirectionMode` Enum**: Added to define `kClientToServer` (0) and `kServerToClient` (1).
- **`thread_Settings` Struct**: Added `mDirection` field of type `DirectionMode`.

## 2. Command-Line Parsing (`src/Settings.cpp`)
- **Flag Definition**: Added `reverse` (long) and `-2` (short).
- **Option Handling**: In `Settings_Interpret`, `-2` sets `mDirection = kServerToClient`.
- **Logic**: The flag is independent of `-s` (server) or `-c` (client).

## 3. Role Execution Logic

### Client-Side (`src/Client.cpp`)
- In `Client::Run()`, the code checks `mDirection`.
- If `kServerToClient`, it calls `data_recv()`.
- Otherwise, it calls `data_send()` (normal behavior).

### Server-Side (`src/Server.cpp`)
- In `Server::Run()`, the code checks `mDirection`.
- If `kServerToClient`, it calls `data_send()`.
- Otherwise, it calls `data_recv()` (normal behavior).

## 4. Data Transfer Helpers (`src/Dataxfer.cpp`)
- **`data_send()`**: Shared logic to transmit data.
- **`data_recv()`**: Shared logic to receive data.
- These functions were modified to handle both TCP and UDP contexts while respecting the `thread_Settings`.

## 5. Summary of Divergence
| Feature | Stock iperf 2.0.5 | This Fork |
| --- | --- | --- |
| `--reverse` Flag | None | Supported (`-2`) |
| Traffic Direction | Fixed (Client -> Server) | Configurable |
| Control Path | Client initiates | Client initiates (Always) |
| Data Path | Follows Control Path | Can be Reversed |

## 6. Limitations Identified
- **UDP Reverse**: Currently non-functional in this fork because the server cannot learn the client's UDP endpoint without an initial packet from the client (which is not sent in reverse mode).
- **Protocol compatibility**: The reverse mode uses the standard iperf2 control header but swaps roles immediately after connection establishment.

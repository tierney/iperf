# Engineering Audit Notes

This document captures technical observations, risks, and protocol details for the `iperf` legacy fork.

## 1. Memory Safety Audit

### Buffer Overflows (Fixed Strings)
- The codebase uses several global or static buffers (e.g., `buffer` in `ReportDefault.c`) with `sprintf` or `strcpy`.
- **Observation**: Many instances were recently updated to use `%s` format strings to mitigate `-Wformat-security` issues.
- **Risk**: Fixed-size buffers (e.g., `REPORT_ADDRLEN` for IP addresses) might overflow if hostname resolution returns extremely long strings.
- **Recommendation**: Audit `inet_ntop` calls to ensure the destination buffer is always `REPORT_ADDRLEN`.

### Memory Management
- The `thread_Settings` struct is frequently allocated with `new` and freed with `Settings_Destroy`.
- **Observation**: In `Reporter.c`, `ReportHeader` is allocated with `malloc` and freed with `free`. This mixing of C/C++ memory styles is common in this 2.x codebase but requires care to avoid leak/mismatch.
- **Finding**: `Settings_Interpret` uses `new char[]` for hostnames.

## 2. Socket & Protocol Behavior

### Reverse Role Swap (TCP)
1. **Control Phase**: Client connects to Server. Client sends `client_hdr` (24-36 bytes).
2. **Negotiation**: Server reads the header. In this fork, if the header or settings indicate `--reverse`, both endpoints swap roles.
3. **Data Phase**:
   - If `kServerToClient`:
     - Server calls `data_send()`.
     - Client calls `data_recv()`.
   - The connection remains the same; only the direction of the application-level `write`/`read` loop is flipped.

### UDP Reverse Limitation
- **Finding**: UDP reverse mode fails because the Server starts calling `data_send()` immediately after the control packet (if TCP) or assuming the client is at the same IP/Port.
- **Protocol Gap**: Standard iperf2 UDP requires the client to send a packet to "open" the server's receive state. In reverse mode, if the client waits for data without sending first, the server has no destination IP/Port in its `mSock` unless it was bound during a previous packet.
- **Conclusion**: This fork's reverse mode is primarily intended for TCP.

## 3. Signal Handling
- The code uses a global `sInterupted` flag.
- **Observation**: It is used in `data_send`/`data_recv` loops to break out on Ctrl+C.
- **Legacy Debt**: Signal handlers are established in `main.cpp` using `signal()`. On modern systems, `sigaction` would be preferred for consistency.

## 4. Integer Overflows
- Bandwidth calculations use `max_size_t` (defined in `headers.h`).
- On 64-bit systems, this is 64-bit, which handles modern multi-gigabit speeds.
- On 32-bit systems, `TotalLen` calculations might overflow for long-running tests.

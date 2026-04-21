# Project Status

This repository is maintained as a **historical iperf2-era fork** with focused modernization:

- preserve and document the reverse NAT/client-initiated behavior patch
- keep builds reproducible on modern systems
- provide characterization tests for behavior comparison
- document migration to stock upstream tools when equivalent behavior exists

Current direction:

1. Preserve legacy behavior and provenance.
2. Add parameterized behavioral comparison against stock `iperf3`.
3. Recommend `iperf3 -R` for forward use when requirements are satisfied by upstream.

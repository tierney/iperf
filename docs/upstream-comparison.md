# Upstream Comparison

## 1. What this fork inherits from iperf 2.x

- iperf2-era code layout and CLI model
- autotools-based build system
- iperf2 protocol family behavior

## 2. What this fork uniquely adds

- reverse option wired as `--reverse` / `-2`
- direction state via `DirectionMode` and `mDirection`
- role-swapped send/receive execution in client/server paths

## 3. What current ESnet iperf3 does differently

- separate implementation and protocol from iperf2
- reverse mode via `-R/--reverse` on the client
- optional `--bidir` for simultaneous bidirectional transfer

## 4. ESnet/iperf practices worth adopting here

- clearer project status and migration documentation
- reproducible CI workflows and smoke tests
- explicit behavior contracts in docs

## 5. ESnet/iperf behaviors not to adopt in this legacy tree

- do not claim iperf2/iperf3 protocol compatibility
- do not rewrite output semantics solely to mirror iperf3
- do not replace the repository identity with an iperf3 fork

## 6. Supersedence assessment

For the original NAT'd client reverse-throughput use case, modern stock `iperf3 -R` is generally sufficient.
This repository remains valuable as historical implementation evidence and for side-by-side behavior verification.

## 7. Recommended long-term path

- keep this repo as a historical, documented fork
- maintain parameterized comparison tests
- point forward to stock `iperf3` when tests confirm requirements are met

# iperf3 Strategy

## Goal

Determine whether stock `iperf3` already satisfies the original purpose of this legacy fork:
client-initiated reverse TCP measurement when clients may be behind NAT/firewalls.

## Working conclusion

Use stock `iperf3 -R` as the forward path when behavioral requirements are met.
Keep this repository as historical reference and comparison harness.

## Comparison matrix

| Capability | Legacy fork | Stock iperf3 |
|---|---|---|
| Normal TCP test | `iperf -s` / `iperf -c host` | `iperf3 -s` / `iperf3 -c host` |
| Reverse TCP test | `iperf -s --reverse` and `iperf -c host --reverse` | `iperf3 -s` and `iperf3 -c host -R` |
| Bidirectional simultaneous | legacy `-d`/`-r` family | `iperf3 --bidir` |

## How to demonstrate equivalence locally

Install `iperf3` if needed:

```sh
# macOS
brew install iperf3

# Ubuntu/Debian
sudo apt-get update && sudo apt-get install -y iperf3
```

Run the parameterized matrix:

```sh
./tests/reverse-matrix.sh
```

Use suite filters:

```sh
RUN_LEGACY=0 ./tests/reverse-matrix.sh
RUN_IPERF3=0 ./tests/reverse-matrix.sh
```

If troubleshooting, keep logs:

```sh
KEEP_TMP=1 FAIL_ON_MISSING_BIN=1 ./tests/reverse-matrix.sh
```

Adjust binaries and network settings as needed:

```sh
LEGACY_IPERF_BIN=/path/to/legacy/iperf \
IPERF3_BIN=/usr/local/bin/iperf3 \
PORT_BASE=56001 \
./tests/reverse-matrix.sh
```

## Decision rule

- If required reverse behaviors pass with stock `iperf3`, documentation should recommend `iperf3` first.
- If a concrete requirement fails in stock `iperf3`, document the exact gap and reproduce it in the matrix before considering any fork/wrapper work.

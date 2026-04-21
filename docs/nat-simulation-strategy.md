# NAT Simulation Strategy

Since local Docker or Linux network namespaces are unavailable in this environment, this document outlines how to manually or scriptedly verify the Reverse-NAT behavior using external tools.

## 1. The Core Scenario

The primary use case for this fork is:
- **Server**: Publicly reachable IP (e.g., `1.1.1.1`).
- **Client**: Behind a NAT/Firewall (e.g., `192.168.1.100`).
- **Constraint**: The Firewall allows outbound connections but blocks all inbound unsolicited connections to the client.

## 2. Simulation Setup (External)

If you have access to a Linux host with Docker, use the following `docker-compose.yml` pattern:

```yaml
version: '3'
services:
  server:
    image: alpine:latest
    command: /bin/sh -c "apk add build-base && ./configure && make && ./src/iperf -s --reverse"
    networks:
      public_net:
        ipv4_address: 10.0.0.10

  nat:
    image: alpine:latest
    privileged: true
    command: /bin/sh -c "iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE && tail -f /dev/null"
    networks:
      public_net:
      private_net:

  client:
    image: alpine:latest
    command: /bin/sh -c "apk add build-base && ./configure && make && ./src/iperf -c 10.0.0.10 --reverse"
    networks:
      private_net:
```

## 3. Local Verification (macOS Alternative)

On macOS, you can simulate the constraint using `pf` (Packet Filter).

### Step 1: Start Server
```sh
./src/iperf -s --reverse -p 55000
```

### Step 2: Block Inbound to a specific port
(Requires sudo)
```sh
echo "block in proto tcp from any to any port 55000" | sudo pfctl -ef -
```

### Step 3: Verify Normal Test Fails (if client were server)
If the client tried to run `iperf -s` and the server tried to connect, it would fail.

### Step 4: Verify Reverse Test Succeeds
```sh
./src/iperf -c 127.0.0.1 -p 55000 --reverse
```
Since the client initiates the connection (outbound), `pf` allows the stateful connection and the data flow in reverse.

## 4. Expected Results
- **Normal**: Fails if the NAT/Firewall blocks the incoming data connection.
- **Reverse**: Succeeds because the control and data paths both utilize the client-initiated stateful tunnel.

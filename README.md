# BioCAN — Organ Network Simulation over SocketCAN

A simulated three-node CAN network (Brain, Heart, Lungs) that coordinates
over a virtual CAN bus, demonstrates priority-based arbitration, detects
node failures via heartbeat timeout (network-management style), sets
DTCs on fault, and exposes a minimal UDS/ISO-TP diagnostic interface per
node. Framed as a physiology analogy — not a claim to model real biology.

## Architecture

```
                         vcan0 (virtual CAN bus)
        ┌───────────────────┬───────────────────┬───────────────────┐
        │                   │                    │                   │
   [BRAIN node]        [HEART node]         [LUNGS node]        [TESTER]
   0x080 CMD           0x100 STATUS         0x180 STATUS        UDS client
   0x081 Heartbeat      (also its own                            over ISO-TP
   0x001 Emergency       heartbeat)
   (highest prio)
```

Lower CAN ID = higher bus priority (standard CAN bitwise arbitration).
`ID_EMERGENCY_ALERT` (0x001) will always win contention against routine
telemetry — this is demonstrated, not just asserted, in Step 3.

## CAN ID Map

| ID    | Name              | Sender | Purpose                          |
|-------|-------------------|--------|-----------------------------------|
| 0x001 | EMERGENCY_ALERT   | any    | Critical fault broadcast (highest priority) |
| 0x080 | BRAIN_CMD         | Brain  | Control command to Heart/Lungs   |
| 0x081 | BRAIN_HEARTBEAT   | Brain  | Keep-alive, 100ms period          |
| 0x100 | HEART_STATUS      | Heart  | HR + BP telemetry, 100ms period (doubles as heartbeat) |
| 0x180 | LUNGS_STATUS      | Lungs  | SpO2 + resp rate, 100ms period    |
| 0x7A0/0x7A8 | BRAIN diag req/resp | Brain/Tester | UDS diagnostic session |
| 0x7B0/0x7B8 | HEART diag req/resp | Heart/Tester | UDS diagnostic session |
| 0x7C0/0x7C8 | LUNGS diag req/resp | Lungs/Tester | UDS diagnostic session |

Full byte-level payload layout is documented in `canids.h`, and mirrored
in `physio.dbc` for Python-side decoding via `cantools`.

## DTCs

| Code   | Meaning                | Trigger condition                          |
|--------|-------------------------|---------------------------------------------|
| C0001  | Cardiac Arrest Detected | HEART_STATUS frame missing > 350ms          |
| C0002  | Hypoxia Detected        | SpO2 < threshold, or LUNGS_STATUS stale     |
| C0003  | CNS Signal Loss         | BRAIN_HEARTBEAT frame missing > 350ms       |

## Environment Setup (Ubuntu, native or VM)

```bash
sudo apt update
sudo apt install -y can-utils build-essential gcc make python3 python3-pip
sudo apt install -y linux-modules-extra-$(uname -r)
pip3 install python-can cantools

sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# sanity check
candump vcan0 &
cansend vcan0 123#DEADBEEF
```

## Roadmap

- [x] Step 1 — CAN ID map, message layout, DTC table (this file + canids.h + physio.dbc)
- [x] Step 2 — `heart.c` and `lungs.c`: periodic telemetry senders
- [ ] Step 3 — `brain.c`: reads Heart/Lungs telemetry, issues commands, demonstrates arbitration under contention
- [ ] Step 4 — Heartbeat timeout detection + DTC setting + EMERGENCY_ALERT broadcast
- [ ] Step 5 — Fault injection (kill a node / stale sensor) to trigger each DTC
- [ ] Step 6 — Minimal UDS/ISO-TP diagnostic server in each node (0x22 / 0x19 / 0x14)
- [ ] Step 7 — Tester tool (C or Python) acting as the diagnostic client
- [ ] Step 8 (optional stretch) — trace logging in a CANoe-like format

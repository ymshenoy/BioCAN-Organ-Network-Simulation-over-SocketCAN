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

| Code   | Meaning                | Trigger condition                          | Detected by  |
|--------|-------------------------|---------------------------------------------|--------------|
| C0001  | Cardiac Arrest Detected | HEART_STATUS frame missing > 350ms          | Brain        |
| C0002  | Hypoxia Detected        | SpO2 < threshold, stale-sample flag, or LUNGS_STATUS frame missing > 350ms | Brain |
| C0003  | CNS Signal Loss         | BRAIN_HEARTBEAT frame missing > 350ms       | Heart, Lungs (each independently) |

Each DTC is edge-triggered: it's set (and `EMERGENCY_ALERT` broadcast) once when the
condition first becomes true, and cleared once telemetry/heartbeat resumes — a node
doesn't re-broadcast every cycle while the fault is still active.

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

## Testing the DTCs (fault injection)

```bash
gcc -o heart heart.c -lm
gcc -o lungs lungs.c -lm
gcc -o brain brain.c

candump vcan0 &          # terminal for watching bus traffic, incl. EMERGENCY_ALERT

./heart & echo "heart pid: $!"
./lungs & echo "lungs pid: $!"
./brain &

# C0001 Cardiac Arrest — kill Heart, wait > 350ms, Brain sets the DTC
kill -9 <heart_pid>

# C0003 CNS Signal Loss — kill Brain, wait > 350ms, Heart and Lungs each set the DTC
kill -9 <brain_pid>

# C0002 Hypoxia — toggle a simulated stuck/hypoxic sensor on a *running* Lungs,
# without killing the process (proves the threshold/flag path, not just timeout)
kill -USR1 <lungs_pid>   # inject fault: SpO2 forced low + stale flag set
kill -USR1 <lungs_pid>   # clear fault
```

## Testing the diagnostic server (UDS/ISO-TP)

Each node listens for single-frame ISO-TP requests on its own `*_DIAG_REQ` ID
and replies on `*_DIAG_RESP`. Frame format: byte 0 = ISO-TP PCI (`0x0` nibble
+ payload length), followed by the UDS payload (SID + params).

```bash
candump vcan0 &

# ReadDataByIdentifier (0x22) — Heart's heart rate (DID 0xF010)
# request: 03 22 F0 10 -> len=3, SID=0x22, DID=0xF010
cansend vcan0 7B0#0322F010
# response on 0x7B8: 05 62 F0 10 <hr_lo> <hr_hi>

# ReadDataByIdentifier — Lungs' SpO2 (DID 0xF020)
cansend vcan0 7C0#0322F020
# response on 0x7C8: 04 62 F0 20 <spo2>

# ReadDataByIdentifier — Brain's status bitmask (DID 0xF030)
cansend vcan0 7A0#0322F030
# response on 0x7A8: 04 62 F0 30 <bit0=cardiac_arrest, bit1=hypoxia>

# ReadDTCInformation (0x19), sub-function 0x02 (reportDTCByStatusMask)
cansend vcan0 7B0#021902
# response on 0x7B8: 59 02 [<dtc_hi> <dtc_lo> <status>]... (empty list if no DTC active)

# ClearDiagnosticInformation (0x14), group = all (0xFFFFFF)
cansend vcan0 7B0#0414FFFFFF
# response on 0x7B8: 01 54
```

An unsupported DID gets a negative response (`7F <SID> 31` = requestOutOfRange);
an unsupported SID gets `7F <SID> 10` (generalReject). A proper `tester.c`/
Python client that builds these frames programmatically is Step 7.

## Roadmap

- [x] Step 1 — CAN ID map, message layout, DTC table (this file + canids.h + physio.dbc)
- [x] Step 2 — `heart.c` and `lungs.c`: periodic telemetry senders
- [x] Step 3a — `brain.c`: reads Heart/Lungs telemetry, issues commands
- [ ] Step 3b — `brain.c`: demonstrates arbitration under contention (vcan doesn't model real bitwise arbitration — needs a bus-load/`cangen` based approach)
- [x] Step 4 — Heartbeat timeout detection + DTC setting + EMERGENCY_ALERT broadcast
- [x] Step 5 — Fault injection (kill a node / stale sensor) to trigger each DTC
- [x] Step 6 — Minimal UDS/ISO-TP diagnostic server in each node (0x22 / 0x19 / 0x14)
- [ ] Step 7 — Tester tool (C or Python) acting as the diagnostic client

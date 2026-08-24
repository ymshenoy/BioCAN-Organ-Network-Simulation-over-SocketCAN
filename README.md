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
gcc -o tester tester.c

candump vcan0 &          # terminal for watching bus traffic, incl. EMERGENCY_ALERT

./heart & echo "heart pid: $!"
./lungs & echo "lungs pid: $!"
./brain & echo "brain pid: $!"

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

`tester` is a small CLI client (`tester.c`) that builds the request, sends it,
and decodes the response — no manual hex needed:

```bash
./tester heart read heart_rate       # ReadDataByIdentifier -> "-> 78 bpm"
./tester heart read blood_pressure   # -> "-> 120/80 mmHg"
./tester lungs read spo2             # -> "-> 98%"
./tester lungs read resp_rate        # -> "-> 16 breaths/min"
./tester brain read brain_status     # -> "-> cardiac_arrest=0 hypoxia=0"
./tester heart read F010             # raw hex DID also accepted

./tester heart dtc                   # ReadDTCInformation
./tester heart clear                 # ClearDiagnosticInformation (all groups)
```

Interface defaults to `vcan0`; override with `IFACE=vcan1 ./tester ...`.
An unsupported DID gets a negative response (`SID=0x22 NRC=0x31` =
requestOutOfRange); an unsupported service gets `NRC=0x10` (generalReject).

Equivalent raw form with `can-utils`, if you want to see the bytes on the wire:

```bash
candump vcan0 &

# ReadDataByIdentifier (0x22) — Heart's heart rate (DID 0xF010)
# request: 03 22 F0 10 -> len=3, SID=0x22, DID=0xF010
cansend vcan0 7B0#0322F010
# response on 0x7B8: 05 62 F0 10 <hr_lo> <hr_hi>

# ReadDTCInformation (0x19), sub-function 0x02 (reportDTCByStatusMask)
cansend vcan0 7B0#021902
# response on 0x7B8: 59 02 [<dtc_hi> <dtc_lo> <status>]... (empty list if no DTC active)

# ClearDiagnosticInformation (0x14), group = all (0xFFFFFF)
cansend vcan0 7B0#0414FFFFFF
# response on 0x7B8: 01 54
```

## Bus-load demo (Step 3b)

`vcan` is a software loopback interface, not a shared electrical bus — it
doesn't perform real bitwise arbitration, so "lower ID always wins" can't be
demonstrated on it the way it would be on physical CAN hardware. What *can*
be shown is that `vcan` never makes `ID_EMERGENCY_ALERT` (0x001) wait behind
queued low-priority traffic, since there's no queue to get stuck behind:

```bash
./arbitration_demo.sh          # floods vcan0 with cangen, then sends 0x001 mid-flood
```

Watch the `candump` output it starts: the `001` frame appears immediately
after the "Sending EMERGENCY_ALERT" marker, undelayed by the surrounding
0x7FF flood. See the script's header comment for why this is a different
(and weaker) claim than real bitwise arbitration.


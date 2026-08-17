/*
 * brain.c — BioCAN Brain node
 *
 * Reads HEART_STATUS and LUNGS_STATUS telemetry off vcan0, sends its own
 * 100ms BRAIN_HEARTBEAT, issues BRAIN_CMD on simple telemetry thresholds,
 * and watches for heartbeat timeouts to set DTCs and broadcast
 * EMERGENCY_ALERT:
 *   - HEART_STATUS missing > 350ms                         -> C0001 Cardiac Arrest
 *   - SpO2 below threshold, stale-sample flag, or
 *     LUNGS_STATUS missing > 350ms                         -> C0002 Hypoxia
 *
 * C0003 CNS Signal Loss (BRAIN_HEARTBEAT missing) is detected by
 * heart.c/lungs.c instead, since they're the ones watching for it.
 *
 * Also runs a minimal UDS/ISO-TP diagnostic server on ID_BRAIN_DIAG_REQ /
 * ID_BRAIN_DIAG_RESP (0x7A0/0x7A8): ReadDataByIdentifier (0x22) for
 * DID_BRAIN_STATUS (a bitmask: bit0 = cardiac arrest active, bit1 =
 * hypoxia active), plus ReadDTCInformation (0x19) and
 * ClearDiagnosticInformation (0x14) for the two DTCs Brain detects
 * (C0001, C0002). Only single-frame ISO-TP is implemented — every message
 * here fits in one CAN frame (<=7 payload bytes), so no multi-frame
 * flow-control state machine is needed.
 *
 * The arbitration-under-contention demo (Step 3b) is not in this file —
 * see README roadmap.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "canids.h"

#define SPO2_HYPOXIA_THRESHOLD      92   /* % — below this, ask Lungs to breathe faster */
#define BP_SYSTOLIC_LOW_THRESHOLD   100  /* mmHg — below this, ask Heart to vasoconstrict */

static int open_can_socket(const char *ifname)
{
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket");
        exit(1);
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl(SIOCGIFINDEX)");
        exit(1);
    }

    struct sockaddr_can addr = {0};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        exit(1);
    }

    return s;
}

/* ms remaining until target, clamped to 0 (never negative — select() needs that).
 * Also doubles as "ms elapsed since": ms_until(&now, &earlier) == now - earlier. */
static long ms_until(const struct timespec *target, const struct timespec *now)
{
    long ms = (target->tv_sec - now->tv_sec) * 1000L
            + (target->tv_nsec - now->tv_nsec) / 1000000L;
    return ms < 0 ? 0 : ms;
}

static void advance_by_ms(struct timespec *t, long ms)
{
    t->tv_nsec += ms * 1000000L;
    t->tv_sec  += t->tv_nsec / 1000000000L;
    t->tv_nsec %= 1000000000L;
}

static void send_emergency_alert(int s, uint8_t source_organ, uint16_t dtc)
{
    struct can_frame alert = {0};
    alert.can_id = ID_EMERGENCY_ALERT;
    alert.can_dlc = 2;
    alert.data[0] = source_organ;
    alert.data[1] = dtc & 0xFF;
    if (write(s, &alert, sizeof(alert)) != sizeof(alert)) {
        perror("write emergency alert");
    }
}

/* ISO-TP Single Frame: PCI byte = 0x0 nibble | length, followed by up to 7
 * payload bytes. That's the whole frame format we need here. */
static void send_iso_tp_sf(int s, canid_t resp_id, const uint8_t *payload, int len)
{
    struct can_frame frame = {0};
    frame.can_id = resp_id;
    frame.can_dlc = len + 1;
    frame.data[0] = (uint8_t)(len & 0x0F);
    memcpy(&frame.data[1], payload, len);
    if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("write diag resp");
    }
}

static void send_negative_response(int s, canid_t resp_id, uint8_t sid, uint8_t nrc)
{
    uint8_t payload[3] = { 0x7F, sid, nrc };
    send_iso_tp_sf(s, resp_id, payload, 3);
}

/* Brain's UDS server: knows DID_BRAIN_STATUS (a bitmask of the two DTCs it
 * detects) and can report/clear those DTCs (C0001, C0002). It does not
 * expose C0003, since it can't detect its own missing heartbeat. */
static void handle_diag_request(int s, const struct can_frame *req,
                                 int *cardiac_arrest_active, int *hypoxia_active)
{
    if (req->can_dlc < 1) return;
    uint8_t pci = req->data[0];
    if ((pci & 0xF0) != 0x00) return;      /* only single-frame requests supported */
    int len = pci & 0x0F;
    if (len < 1 || len > 7) return;
    uint8_t sid = req->data[1];

    switch (sid) {
    case UDS_READ_DATA_BY_ID: {
        if (len < 3) {
            send_negative_response(s, ID_BRAIN_DIAG_RESP, sid, UDS_NRC_GENERAL_REJECT);
            break;
        }
        uint16_t did = ((uint16_t)req->data[2] << 8) | req->data[3];
        if (did != DID_BRAIN_STATUS) {
            send_negative_response(s, ID_BRAIN_DIAG_RESP, sid, UDS_NRC_REQUEST_OUT_OF_RANGE);
            break;
        }
        uint8_t status = (*cardiac_arrest_active ? 0x01 : 0x00)
                        | (*hypoxia_active ? 0x02 : 0x00);
        uint8_t resp[4] = { UDS_READ_DATA_BY_ID + 0x40, did >> 8, did & 0xFF, status };
        send_iso_tp_sf(s, ID_BRAIN_DIAG_RESP, resp, 4);
        break;
    }
    case UDS_READ_DTC_INFO: {
        uint8_t subfunction = (len >= 2) ? req->data[2] : 0x02;
        uint8_t resp[8];
        resp[0] = UDS_READ_DTC_INFO + 0x40;
        resp[1] = subfunction;
        int rlen = 2;
        if (*cardiac_arrest_active) {
            resp[rlen++] = (DTC_C0001_CARDIAC_ARREST >> 8) & 0xFF;
            resp[rlen++] = DTC_C0001_CARDIAC_ARREST & 0xFF;
            resp[rlen++] = 0xFF; /* status: active */
        }
        if (*hypoxia_active) {
            resp[rlen++] = (DTC_C0002_HYPOXIA >> 8) & 0xFF;
            resp[rlen++] = DTC_C0002_HYPOXIA & 0xFF;
            resp[rlen++] = 0xFF; /* status: active */
        }
        send_iso_tp_sf(s, ID_BRAIN_DIAG_RESP, resp, rlen);
        break;
    }
    case UDS_CLEAR_DTC: {
        *cardiac_arrest_active = 0;
        *hypoxia_active = 0;
        uint8_t resp[1] = { UDS_CLEAR_DTC + 0x40 };
        send_iso_tp_sf(s, ID_BRAIN_DIAG_RESP, resp, 1);
        break;
    }
    default:
        send_negative_response(s, ID_BRAIN_DIAG_RESP, sid, UDS_NRC_GENERAL_REJECT);
        break;
    }
}

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "vcan0";
    int s = open_can_socket(ifname);

    int heart_rate = 0, bp_systolic = 0, bp_diastolic = 0;
    int spo2 = 0, resp_rate = 0, lungs_stale = 0;
    int heart_seen = 0, lungs_seen = 0;
    int cardiac_arrest_active = 0, hypoxia_active = 0;
    struct timespec last_heart_rx = {0}, last_lungs_rx = {0};

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick);
    advance_by_ms(&next_tick, HEARTBEAT_PERIOD_MS);

    while (1) {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        long wait_ms = ms_until(&next_tick, &now);

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s, &rfds);
        struct timeval tv;
        tv.tv_sec  = wait_ms / 1000;
        tv.tv_usec = (wait_ms % 1000) * 1000;

        int ready = select(s + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            perror("select");
            exit(1);
        }

        if (ready > 0 && FD_ISSET(s, &rfds)) {
            struct can_frame frame;
            ssize_t n = read(s, &frame, sizeof(frame));
            if (n != sizeof(frame)) {
                perror("read");
            } else {
                struct timespec rx_time;
                clock_gettime(CLOCK_MONOTONIC, &rx_time);

                switch (frame.can_id) {
                case ID_HEART_STATUS:
                    heart_rate    = frame.data[0] | (frame.data[1] << 8);
                    bp_systolic   = frame.data[2];
                    bp_diastolic  = frame.data[3];
                    heart_seen    = 1;
                    last_heart_rx = rx_time;
                    printf("[BRAIN] rx HEART_STATUS  HR=%3d bpm  BP=%3d/%3d mmHg\n",
                           heart_rate, bp_systolic, bp_diastolic);
                    fflush(stdout);
                    break;
                case ID_LUNGS_STATUS:
                    spo2          = frame.data[0];
                    resp_rate     = frame.data[1];
                    lungs_stale   = frame.data[2] & 0x01;
                    lungs_seen    = 1;
                    last_lungs_rx = rx_time;
                    printf("[BRAIN] rx LUNGS_STATUS  SpO2=%3d%%  RespRate=%2d breaths/min\n",
                           spo2, resp_rate);
                    fflush(stdout);
                    break;
                case ID_BRAIN_DIAG_REQ:
                    handle_diag_request(s, &frame, &cardiac_arrest_active, &hypoxia_active);
                    break;
                default:
                    break; /* not telemetry Brain acts on (e.g. its own looped-back frames) */
                }
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        if (ms_until(&next_tick, &now) == 0) {
            uint8_t command = CMD_NONE;
            if (lungs_seen && spo2 < SPO2_HYPOXIA_THRESHOLD) {
                command = CMD_INCREASE_RESP_RATE;
            } else if (heart_seen && bp_systolic < BP_SYSTOLIC_LOW_THRESHOLD) {
                command = CMD_VASOCONSTRICT;
            }

            struct can_frame hb = {0};
            hb.can_id = ID_BRAIN_HEARTBEAT;
            hb.can_dlc = 1;
            hb.data[0] = 1; /* Alive */
            if (write(s, &hb, sizeof(hb)) != sizeof(hb)) {
                perror("write heartbeat");
            }

            if (command != CMD_NONE) {
                struct can_frame cmd = {0};
                cmd.can_id = ID_BRAIN_CMD;
                cmd.can_dlc = 1;
                cmd.data[0] = command;
                if (write(s, &cmd, sizeof(cmd)) != sizeof(cmd)) {
                    perror("write cmd");
                } else {
                    printf("[BRAIN] tx BRAIN_CMD  command=0x%02X\n", command);
                    fflush(stdout);
                }
            }

            if (heart_seen) {
                long since_heart = ms_until(&now, &last_heart_rx);
                if (since_heart > HEARTBEAT_TIMEOUT_MS && !cardiac_arrest_active) {
                    cardiac_arrest_active = 1;
                    printf("[BRAIN] DTC C0001 CARDIAC_ARREST set (HEART_STATUS missing %ldms)\n", since_heart);
                    fflush(stdout);
                    send_emergency_alert(s, ORGAN_HEART, DTC_C0001_CARDIAC_ARREST);
                } else if (since_heart <= HEARTBEAT_TIMEOUT_MS && cardiac_arrest_active) {
                    cardiac_arrest_active = 0;
                    printf("[BRAIN] DTC C0001 CARDIAC_ARREST cleared (HEART_STATUS resumed)\n");
                    fflush(stdout);
                }
            }

            if (lungs_seen) {
                long since_lungs = ms_until(&now, &last_lungs_rx);
                int hypoxia_condition = (spo2 < SPO2_HYPOXIA_THRESHOLD)
                                       || (since_lungs > HEARTBEAT_TIMEOUT_MS)
                                       || lungs_stale;
                if (hypoxia_condition && !hypoxia_active) {
                    hypoxia_active = 1;
                    printf("[BRAIN] DTC C0002 HYPOXIA set (SpO2=%d%% stale=%ldms flag=%d)\n",
                           spo2, since_lungs, lungs_stale);
                    fflush(stdout);
                    send_emergency_alert(s, ORGAN_LUNGS, DTC_C0002_HYPOXIA);
                } else if (!hypoxia_condition && hypoxia_active) {
                    hypoxia_active = 0;
                    printf("[BRAIN] DTC C0002 HYPOXIA cleared\n");
                    fflush(stdout);
                }
            }

            advance_by_ms(&next_tick, HEARTBEAT_PERIOD_MS);
        }
    }

    close(s);
    return 0;
}

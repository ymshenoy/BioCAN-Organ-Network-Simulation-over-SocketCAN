/*
 * heart.c — BioCAN Heart node
 *
 * Periodically broadcasts HEART_STATUS (0x100) on vcan0 with simulated
 * heart rate and blood pressure. This frame also serves as Heart's
 * heartbeat — Brain declares CARDIAC_ARREST if it goes missing > 350ms.
 *
 * Also watches BRAIN_HEARTBEAT (0x081): if it goes missing > 350ms, Heart
 * sets DTC C0003 CNS_SIGNAL_LOSS and broadcasts EMERGENCY_ALERT itself.
 *
 * Also runs a minimal UDS/ISO-TP diagnostic server on ID_HEART_DIAG_REQ /
 * ID_HEART_DIAG_RESP (0x7B0/0x7B8): ReadDataByIdentifier (0x22) for
 * DID_HEART_RATE/DID_BLOOD_PRESSURE, plus ReadDTCInformation (0x19) and
 * ClearDiagnosticInformation (0x14) for Heart's own DTC (C0003). Only
 * single-frame ISO-TP is implemented — every message here fits in one CAN
 * frame (<=7 payload bytes), so no multi-frame flow-control state machine
 * is needed.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "canids.h"

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

/* Heart's UDS server: knows only its own telemetry (heart_rate/bp_*) and its
 * own DTC (C0003 — Heart is the one watching for BRAIN_HEARTBEAT timeout). */
static void handle_diag_request(int s, const struct can_frame *req,
                                 int heart_rate, int bp_systolic, int bp_diastolic,
                                 int *cns_signal_loss_active)
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
            send_negative_response(s, ID_HEART_DIAG_RESP, sid, UDS_NRC_GENERAL_REJECT);
            break;
        }
        uint16_t did = ((uint16_t)req->data[2] << 8) | req->data[3];
        uint8_t resp[8];
        resp[0] = UDS_READ_DATA_BY_ID + 0x40;
        resp[1] = (uint8_t)(did >> 8);
        resp[2] = (uint8_t)(did & 0xFF);
        int rlen;
        if (did == DID_HEART_RATE) {
            resp[3] = heart_rate & 0xFF;         /* same byte order as HEART_STATUS */
            resp[4] = (heart_rate >> 8) & 0xFF;
            rlen = 5;
        } else if (did == DID_BLOOD_PRESSURE) {
            resp[3] = bp_systolic & 0xFF;
            resp[4] = bp_diastolic & 0xFF;
            rlen = 5;
        } else {
            send_negative_response(s, ID_HEART_DIAG_RESP, sid, UDS_NRC_REQUEST_OUT_OF_RANGE);
            break;
        }
        send_iso_tp_sf(s, ID_HEART_DIAG_RESP, resp, rlen);
        break;
    }
    case UDS_READ_DTC_INFO: {
        uint8_t subfunction = (len >= 2) ? req->data[2] : 0x02;
        uint8_t resp[8];
        resp[0] = UDS_READ_DTC_INFO + 0x40;
        resp[1] = subfunction;
        int rlen = 2;
        if (*cns_signal_loss_active) {
            resp[rlen++] = (DTC_C0003_CNS_SIGNAL_LOSS >> 8) & 0xFF;
            resp[rlen++] = DTC_C0003_CNS_SIGNAL_LOSS & 0xFF;
            resp[rlen++] = 0xFF; /* status: active */
        }
        send_iso_tp_sf(s, ID_HEART_DIAG_RESP, resp, rlen);
        break;
    }
    case UDS_CLEAR_DTC: {
        *cns_signal_loss_active = 0;
        uint8_t resp[1] = { UDS_CLEAR_DTC + 0x40 };
        send_iso_tp_sf(s, ID_HEART_DIAG_RESP, resp, 1);
        break;
    }
    default:
        send_negative_response(s, ID_HEART_DIAG_RESP, sid, UDS_NRC_GENERAL_REJECT);
        break;
    }
}

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "vcan0";
    int s = open_can_socket(ifname);

    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    double t = 0.0;
    int heart_rate = 0, bp_systolic = 0, bp_diastolic = 0;
    int brain_hb_seen = 0;
    int cns_signal_loss_active = 0;
    struct timespec last_brain_hb_rx = {0};

    struct timespec next_tick;
    clock_gettime(CLOCK_MONOTONIC, &next_tick); /* fire the first tick immediately */

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
                switch (frame.can_id) {
                case ID_BRAIN_HEARTBEAT:
                    brain_hb_seen = 1;
                    clock_gettime(CLOCK_MONOTONIC, &last_brain_hb_rx);
                    break;
                case ID_HEART_DIAG_REQ:
                    handle_diag_request(s, &frame, heart_rate, bp_systolic, bp_diastolic,
                                         &cns_signal_loss_active);
                    break;
                default:
                    break;
                }
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        if (ms_until(&next_tick, &now) == 0) {
            int jitter_hr = (rand() % 5) - 2;                 /* +/-2 bpm noise      */
            heart_rate = (int)(75 + 15 * sin(t / 5.0)) + jitter_hr;
            if (heart_rate < 40) heart_rate = 40;
            if (heart_rate > 200) heart_rate = 200;

            int jitter_bp = (rand() % 3) - 1;                  /* +/-1 mmHg noise     */
            bp_systolic  = 120 + jitter_bp;
            bp_diastolic = 80 + jitter_bp;

            struct can_frame frame = {0};
            frame.can_id = ID_HEART_STATUS;
            frame.can_dlc = 4;
            frame.data[0] = heart_rate & 0xFF;
            frame.data[1] = (heart_rate >> 8) & 0xFF;
            frame.data[2] = bp_systolic & 0xFF;
            frame.data[3] = bp_diastolic & 0xFF;

            if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
                perror("write");
            } else {
                printf("[HEART] HR=%3d bpm  BP=%3d/%3d mmHg\n", heart_rate, bp_systolic, bp_diastolic);
                fflush(stdout);
            }

            if (brain_hb_seen) {
                long since_hb = ms_until(&now, &last_brain_hb_rx);
                if (since_hb > HEARTBEAT_TIMEOUT_MS && !cns_signal_loss_active) {
                    cns_signal_loss_active = 1;
                    printf("[HEART] DTC C0003 CNS_SIGNAL_LOSS set (BRAIN_HEARTBEAT missing %ldms)\n", since_hb);
                    fflush(stdout);
                    send_emergency_alert(s, ORGAN_BRAIN, DTC_C0003_CNS_SIGNAL_LOSS);
                } else if (since_hb <= HEARTBEAT_TIMEOUT_MS && cns_signal_loss_active) {
                    cns_signal_loss_active = 0;
                    printf("[HEART] DTC C0003 CNS_SIGNAL_LOSS cleared (BRAIN_HEARTBEAT resumed)\n");
                    fflush(stdout);
                }
            }

            advance_by_ms(&next_tick, HEARTBEAT_PERIOD_MS);
            t += HEARTBEAT_PERIOD_MS / 1000.0;
        }
    }

    close(s);
    return 0;
}

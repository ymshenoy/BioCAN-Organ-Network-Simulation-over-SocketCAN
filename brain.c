/*
 * brain.c — BioCAN Brain node
 *
 * Reads HEART_STATUS and LUNGS_STATUS telemetry off vcan0, sends its own
 * 100ms BRAIN_HEARTBEAT, and issues a BRAIN_CMD when telemetry crosses a
 * simple threshold (hypoxia -> increase resp rate, low BP -> vasoconstrict).
 *
 * Heartbeat-timeout fault detection / DTCs / EMERGENCY_ALERT (Step 4) and
 * the arbitration-under-contention demo (Step 3 follow-up) are not in this
 * file yet — see README roadmap.
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

/* ms remaining until target, clamped to 0 (never negative — select() needs that) */
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

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "vcan0";
    int s = open_can_socket(ifname);

    int heart_rate = 0, bp_systolic = 0, bp_diastolic = 0;
    int spo2 = 0, resp_rate = 0;
    int heart_seen = 0, lungs_seen = 0;

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
                switch (frame.can_id) {
                case ID_HEART_STATUS:
                    heart_rate   = frame.data[0] | (frame.data[1] << 8);
                    bp_systolic  = frame.data[2];
                    bp_diastolic = frame.data[3];
                    heart_seen = 1;
                    printf("[BRAIN] rx HEART_STATUS  HR=%3d bpm  BP=%3d/%3d mmHg\n",
                           heart_rate, bp_systolic, bp_diastolic);
                    fflush(stdout);
                    break;
                case ID_LUNGS_STATUS:
                    spo2      = frame.data[0];
                    resp_rate = frame.data[1];
                    lungs_seen = 1;
                    printf("[BRAIN] rx LUNGS_STATUS  SpO2=%3d%%  RespRate=%2d breaths/min\n",
                           spo2, resp_rate);
                    fflush(stdout);
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

            advance_by_ms(&next_tick, HEARTBEAT_PERIOD_MS);
        }
    }

    close(s);
    return 0;
}

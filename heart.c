/*
 * heart.c — BioCAN Heart node
 *
 * Periodically broadcasts HEART_STATUS (0x100) on vcan0 with simulated
 * heart rate and blood pressure. This frame also serves as Heart's
 * heartbeat — Brain declares CARDIAC_ARREST if it goes missing > 350ms.
 *
 * Also watches BRAIN_HEARTBEAT (0x081): if it goes missing > 350ms, Heart
 * sets DTC C0003 CNS_SIGNAL_LOSS and broadcasts EMERGENCY_ALERT itself.
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

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "vcan0";
    int s = open_can_socket(ifname);

    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    double t = 0.0;
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
            } else if (frame.can_id == ID_BRAIN_HEARTBEAT) {
                brain_hb_seen = 1;
                clock_gettime(CLOCK_MONOTONIC, &last_brain_hb_rx);
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &now);
        if (ms_until(&next_tick, &now) == 0) {
            int jitter_hr = (rand() % 5) - 2;                 /* +/-2 bpm noise      */
            int hr = (int)(75 + 15 * sin(t / 5.0)) + jitter_hr;
            if (hr < 40) hr = 40;
            if (hr > 200) hr = 200;

            int jitter_bp = (rand() % 3) - 1;                  /* +/-1 mmHg noise     */
            int systolic  = 120 + jitter_bp;
            int diastolic = 80 + jitter_bp;

            struct can_frame frame = {0};
            frame.can_id = ID_HEART_STATUS;
            frame.can_dlc = 4;
            frame.data[0] = hr & 0xFF;
            frame.data[1] = (hr >> 8) & 0xFF;
            frame.data[2] = systolic & 0xFF;
            frame.data[3] = diastolic & 0xFF;

            if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
                perror("write");
            } else {
                printf("[HEART] HR=%3d bpm  BP=%3d/%3d mmHg\n", hr, systolic, diastolic);
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

/*
 * lungs.c — BioCAN Lungs node
 *
 * Periodically broadcasts LUNGS_STATUS (0x180) on vcan0 with simulated
 * SpO2 and respiration rate. Brain declares HYPOXIA if SpO2 drops below
 * threshold or this frame goes stale.
 *
 * Also watches BRAIN_HEARTBEAT (0x081): if it goes missing > 350ms, Lungs
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
            int jitter_spo2 = (rand() % 3) - 1;                /* +/-1% noise         */
            int spo2 = 98 + jitter_spo2;
            if (spo2 < 0) spo2 = 0;
            if (spo2 > 100) spo2 = 100;

            int jitter_rr = (rand() % 3) - 1;                  /* +/-1 breath noise   */
            int resp_rate = (int)(16 + 2 * sin(t / 7.0)) + jitter_rr;
            if (resp_rate < 0) resp_rate = 0;
            if (resp_rate > 60) resp_rate = 60;

            uint8_t status_flags = 0x00;                       /* bit0 = stale; fresh sample every cycle */

            struct can_frame frame = {0};
            frame.can_id = ID_LUNGS_STATUS;
            frame.can_dlc = 3;
            frame.data[0] = spo2 & 0xFF;
            frame.data[1] = resp_rate & 0xFF;
            frame.data[2] = status_flags;

            if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
                perror("write");
            } else {
                printf("[LUNGS] SpO2=%3d%%  RespRate=%2d breaths/min\n", spo2, resp_rate);
                fflush(stdout);
            }

            if (brain_hb_seen) {
                long since_hb = ms_until(&now, &last_brain_hb_rx);
                if (since_hb > HEARTBEAT_TIMEOUT_MS && !cns_signal_loss_active) {
                    cns_signal_loss_active = 1;
                    printf("[LUNGS] DTC C0003 CNS_SIGNAL_LOSS set (BRAIN_HEARTBEAT missing %ldms)\n", since_hb);
                    fflush(stdout);
                    send_emergency_alert(s, ORGAN_BRAIN, DTC_C0003_CNS_SIGNAL_LOSS);
                } else if (since_hb <= HEARTBEAT_TIMEOUT_MS && cns_signal_loss_active) {
                    cns_signal_loss_active = 0;
                    printf("[LUNGS] DTC C0003 CNS_SIGNAL_LOSS cleared (BRAIN_HEARTBEAT resumed)\n");
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

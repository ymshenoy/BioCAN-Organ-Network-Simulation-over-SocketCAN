/*
 * lungs.c — BioCAN Lungs node
 *
 * Periodically broadcasts LUNGS_STATUS (0x180) on vcan0 with simulated
 * SpO2 and respiration rate. Brain declares HYPOXIA if SpO2 drops below
 * threshold or this frame goes stale.
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

int main(int argc, char **argv)
{
    const char *ifname = (argc > 1) ? argv[1] : "vcan0";
    int s = open_can_socket(ifname);

    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    double t = 0.0;
    while (1) {
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

        usleep(HEARTBEAT_PERIOD_MS * 1000);
        t += HEARTBEAT_PERIOD_MS / 1000.0;
    }

    close(s);
    return 0;
}

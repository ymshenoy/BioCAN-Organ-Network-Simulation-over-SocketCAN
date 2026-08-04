/*
 * heart.c — BioCAN Heart node
 *
 * Periodically broadcasts HEART_STATUS (0x100) on vcan0 with simulated
 * heart rate and blood pressure. This frame also serves as Heart's
 * heartbeat — Brain declares CARDIAC_ARREST if it goes missing > 350ms.
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

        usleep(HEARTBEAT_PERIOD_MS * 1000);
        t += HEARTBEAT_PERIOD_MS / 1000.0;
    }

    close(s);
    return 0;
}

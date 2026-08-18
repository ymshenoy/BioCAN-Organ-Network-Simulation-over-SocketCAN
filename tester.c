/*
 * tester.c — BioCAN diagnostic tester (UDS/ISO-TP client)
 *
 * Command-line UDS client for the diagnostic servers in brain.c/heart.c/
 * lungs.c. Builds a single-frame ISO-TP request, sends it on the target
 * organ's *_DIAG_REQ ID, and decodes whatever arrives on *_DIAG_RESP.
 *
 * Usage:
 *   tester <organ> read <did>     ReadDataByIdentifier (organ-specific DIDs)
 *   tester <organ> dtc            ReadDTCInformation (reportDTCByStatusMask)
 *   tester <organ> clear          ClearDiagnosticInformation (all DTC groups)
 *
 * <organ> is brain|heart|lungs. <did> accepts a friendly name (heart_rate,
 * blood_pressure, spo2, resp_rate, brain_status) or a raw hex identifier
 * (e.g. F010). Interface defaults to vcan0; override with the IFACE
 * environment variable.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <time.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "canids.h"

struct organ_ids {
    const char *name;
    canid_t req_id;
    canid_t resp_id;
};

static const struct organ_ids ORGANS[] = {
    { "brain", ID_BRAIN_DIAG_REQ, ID_BRAIN_DIAG_RESP },
    { "heart", ID_HEART_DIAG_REQ, ID_HEART_DIAG_RESP },
    { "lungs", ID_LUNGS_DIAG_REQ, ID_LUNGS_DIAG_RESP },
};

struct did_name {
    const char *name;
    uint16_t did;
};

static const struct did_name DIDS[] = {
    { "heart_rate",     DID_HEART_RATE },
    { "blood_pressure", DID_BLOOD_PRESSURE },
    { "spo2",           DID_SPO2 },
    { "resp_rate",      DID_RESP_RATE },
    { "brain_status",   DID_BRAIN_STATUS },
};

static void usage(const char *prog)
{
    fprintf(stderr,
        "usage: %s <organ> read <did>   (organ = brain|heart|lungs, did = name or hex)\n"
        "       %s <organ> dtc\n"
        "       %s <organ> clear\n"
        "did names: heart_rate blood_pressure spo2 resp_rate brain_status\n",
        prog, prog, prog);
}

static const struct organ_ids *find_organ(const char *name)
{
    for (size_t i = 0; i < sizeof(ORGANS) / sizeof(ORGANS[0]); i++) {
        if (strcasecmp(name, ORGANS[i].name) == 0) return &ORGANS[i];
    }
    return NULL;
}

static int parse_did(const char *s, uint16_t *out)
{
    for (size_t i = 0; i < sizeof(DIDS) / sizeof(DIDS[0]); i++) {
        if (strcasecmp(s, DIDS[i].name) == 0) { *out = DIDS[i].did; return 1; }
    }
    char *end;
    long v = strtol(s, &end, 16);
    if (*end != '\0' || v < 0 || v > 0xFFFF) return 0;
    *out = (uint16_t)v;
    return 1;
}

static const char *did_name(uint16_t did)
{
    for (size_t i = 0; i < sizeof(DIDS) / sizeof(DIDS[0]); i++) {
        if (DIDS[i].did == did) return DIDS[i].name;
    }
    return "unknown";
}

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

static void send_iso_tp_sf(int s, canid_t req_id, const uint8_t *payload, int len)
{
    struct can_frame frame = {0};
    frame.can_id = req_id;
    frame.can_dlc = len + 1;
    frame.data[0] = (uint8_t)(len & 0x0F);
    memcpy(&frame.data[1], payload, len);
    if (write(s, &frame, sizeof(frame)) != sizeof(frame)) {
        perror("write");
        exit(1);
    }
}

/* Waits up to timeout_ms for a Single Frame response on resp_id, ignoring
 * any other bus traffic (telemetry, other nodes' diag exchanges, etc.). */
static int recv_iso_tp_sf(int s, canid_t resp_id, uint8_t *payload, int *len, int timeout_ms)
{
    struct timespec deadline, now;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    deadline.tv_sec  += timeout_ms / 1000;
    deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) { deadline.tv_sec++; deadline.tv_nsec -= 1000000000L; }

    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &now);
        long remaining_ms = (deadline.tv_sec - now.tv_sec) * 1000L
                           + (deadline.tv_nsec - now.tv_nsec) / 1000000L;
        if (remaining_ms <= 0) return 0;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(s, &rfds);
        struct timeval tv;
        tv.tv_sec  = remaining_ms / 1000;
        tv.tv_usec = (remaining_ms % 1000) * 1000;

        int ready = select(s + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) { perror("select"); return 0; }
        if (ready == 0) return 0; /* timeout */

        struct can_frame frame;
        if (read(s, &frame, sizeof(frame)) != sizeof(frame)) continue;
        if (frame.can_id != resp_id) continue;
        if ((frame.data[0] & 0xF0) != 0x00) continue; /* not a Single Frame */

        *len = frame.data[0] & 0x0F;
        memcpy(payload, &frame.data[1], *len);
        return 1;
    }
}

static const char *nrc_name(uint8_t nrc)
{
    switch (nrc) {
    case UDS_NRC_GENERAL_REJECT:       return "generalReject";
    case UDS_NRC_REQUEST_OUT_OF_RANGE: return "requestOutOfRange";
    default:                           return "unknown";
    }
}

static void print_response(uint8_t sid, const uint8_t *payload, int len)
{
    if (len >= 3 && payload[0] == 0x7F) {
        printf("negative response: SID=0x%02X NRC=0x%02X (%s)\n",
               payload[1], payload[2], nrc_name(payload[2]));
        return;
    }

    if (len < 1 || payload[0] != (uint8_t)(sid + 0x40)) {
        printf("unexpected response (%d bytes):", len);
        for (int i = 0; i < len; i++) printf(" %02X", payload[i]);
        printf("\n");
        return;
    }

    switch (sid) {
    case UDS_READ_DATA_BY_ID: {
        if (len < 3) { printf("malformed RDBI response\n"); break; }
        uint16_t did = ((uint16_t)payload[1] << 8) | payload[2];
        printf("DID 0x%04X (%s):", did, did_name(did));
        for (int i = 3; i < len; i++) printf(" %02X", payload[i]);

        if (did == DID_HEART_RATE && len >= 5) {
            int hr = payload[3] | (payload[4] << 8);
            printf("  -> %d bpm", hr);
        } else if (did == DID_BLOOD_PRESSURE && len >= 5) {
            printf("  -> %d/%d mmHg", payload[3], payload[4]);
        } else if (did == DID_SPO2 && len >= 4) {
            printf("  -> %d%%", payload[3]);
        } else if (did == DID_RESP_RATE && len >= 4) {
            printf("  -> %d breaths/min", payload[3]);
        } else if (did == DID_BRAIN_STATUS && len >= 4) {
            printf("  -> cardiac_arrest=%d hypoxia=%d",
                   payload[3] & 0x01, (payload[3] >> 1) & 0x01);
        }
        printf("\n");
        break;
    }
    case UDS_READ_DTC_INFO: {
        printf("DTCs (sub-function 0x%02X):", len >= 2 ? payload[1] : 0);
        if (len <= 2) {
            printf(" none active\n");
            break;
        }
        printf("\n");
        for (int i = 2; i + 3 <= len; i += 3) {
            uint16_t dtc = ((uint16_t)payload[i] << 8) | payload[i + 1];
            printf("  0x%04X  status=0x%02X\n", dtc, payload[i + 2]);
        }
        break;
    }
    case UDS_CLEAR_DTC:
        printf("DTCs cleared\n");
        break;
    default:
        printf("positive response, %d bytes\n", len);
        break;
    }
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        usage(argv[0]);
        return 1;
    }

    const struct organ_ids *organ = find_organ(argv[1]);
    if (!organ) {
        fprintf(stderr, "unknown organ '%s'\n", argv[1]);
        usage(argv[0]);
        return 1;
    }

    uint8_t req[8];
    int req_len;
    uint8_t sid;

    if (strcasecmp(argv[2], "read") == 0) {
        if (argc < 4) { usage(argv[0]); return 1; }
        uint16_t did;
        if (!parse_did(argv[3], &did)) {
            fprintf(stderr, "unrecognized DID '%s'\n", argv[3]);
            return 1;
        }
        sid = UDS_READ_DATA_BY_ID;
        req[0] = sid; req[1] = (uint8_t)(did >> 8); req[2] = (uint8_t)(did & 0xFF);
        req_len = 3;
    } else if (strcasecmp(argv[2], "dtc") == 0) {
        sid = UDS_READ_DTC_INFO;
        req[0] = sid; req[1] = 0x02; /* reportDTCByStatusMask */
        req_len = 2;
    } else if (strcasecmp(argv[2], "clear") == 0) {
        sid = UDS_CLEAR_DTC;
        req[0] = sid; req[1] = 0xFF; req[2] = 0xFF; req[3] = 0xFF; /* all DTC groups */
        req_len = 4;
    } else {
        fprintf(stderr, "unknown service '%s'\n", argv[2]);
        usage(argv[0]);
        return 1;
    }

    const char *ifname = getenv("IFACE");
    if (!ifname) ifname = "vcan0";
    int s = open_can_socket(ifname);

    send_iso_tp_sf(s, organ->req_id, req, req_len);

    uint8_t resp[8];
    int resp_len;
    if (!recv_iso_tp_sf(s, organ->resp_id, resp, &resp_len, 1000)) {
        fprintf(stderr, "no response from %s within 1000ms\n", argv[1]);
        close(s);
        return 1;
    }

    print_response(sid, resp, resp_len);

    close(s);
    return 0;
}

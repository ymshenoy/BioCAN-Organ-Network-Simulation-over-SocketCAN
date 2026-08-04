/*
 * canids.h — BioCAN shared CAN ID map, message layout, and DTC definitions
 *
 * Priority note: in CAN, a LOWER numeric arbitration ID wins the bus when
 * multiple nodes transmit simultaneously (bitwise arbitration on dominant
 * bit 0). So the IDs below are deliberately ordered low = highest priority,
 * mirroring the idea that a life-threatening emergency should always win
 * bus access over routine telemetry.
 *
 * This is used identically by brain.c, heart.c, lungs.c, and the tester,
 * so all nodes agree on the same wire format.
 */

#ifndef CANIDS_H
#define CANIDS_H

#include <stdint.h>

/* ---------------------------------------------------------------------
 * NORMAL / PERIODIC MESSAGE IDs  (priority order, lowest ID = wins bus)
 * ------------------------------------------------------------------- */

#define ID_EMERGENCY_ALERT   0x001  /* any node: critical fault broadcast   */

#define ID_BRAIN_CMD         0x080  /* brain -> heart/lungs: control cmd    */
#define ID_BRAIN_HEARTBEAT   0x081  /* brain -> bus: "I'm alive" (100 ms)   */

#define ID_HEART_STATUS      0x100  /* heart -> bus: HR + BP (100 ms)       */
#define ID_LUNGS_STATUS      0x180  /* lungs -> bus: SpO2 + resp (100 ms)   */

/* ---------------------------------------------------------------------
 * UDS-STYLE DIAGNOSTIC REQUEST/RESPONSE IDs (ISO-TP, one pair per organ)
 * Convention mirrors real automotive practice: response ID = request + 8
 * ------------------------------------------------------------------- */

#define ID_BRAIN_DIAG_REQ    0x7A0
#define ID_BRAIN_DIAG_RESP   0x7A8

#define ID_HEART_DIAG_REQ    0x7B0
#define ID_HEART_DIAG_RESP   0x7B8

#define ID_LUNGS_DIAG_REQ    0x7C0
#define ID_LUNGS_DIAG_RESP   0x7C8

/* ---------------------------------------------------------------------
 * BRAIN_CMD payload — byte 0 = command_id
 * ------------------------------------------------------------------- */

#define CMD_NONE               0x00
#define CMD_INCREASE_RESP_RATE 0x01
#define CMD_VASOCONSTRICT      0x02
#define CMD_SAFE_MODE          0x03

/* ---------------------------------------------------------------------
 * HEART_STATUS payload (4 bytes used of 8)
 *   byte 0-1 : heart_rate_bpm      (uint16, raw = bpm, range 0-250)
 *   byte 2   : bp_systolic_mmHg    (uint8,  raw = mmHg, range 0-250)
 *   byte 3   : bp_diastolic_mmHg   (uint8,  raw = mmHg, range 0-150)
 * ------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 * LUNGS_STATUS payload (3 bytes used of 8)
 *   byte 0 : spo2_percent          (uint8,  raw = %, range 0-100)
 *   byte 1 : resp_rate_bpm         (uint8,  raw = breaths/min, 0-60)
 *   byte 2 : status_flags          (bit0 = stale/no new sample this cycle)
 * ------------------------------------------------------------------- */

/* ---------------------------------------------------------------------
 * EMERGENCY_ALERT payload — byte 0 = source organ, byte 1 = DTC code
 * ------------------------------------------------------------------- */

#define ORGAN_BRAIN  0x01
#define ORGAN_HEART  0x02
#define ORGAN_LUNGS  0x03

/* ---------------------------------------------------------------------
 * DTCs (Diagnostic Trouble Codes) — automotive-style naming (Cxxxx =
 * "chassis/body" domain code space; borrowed here for the organ domain)
 * ------------------------------------------------------------------- */

#define DTC_NONE                   0x0000
#define DTC_C0001_CARDIAC_ARREST   0xC001  /* heart heartbeat frame missing */
#define DTC_C0002_HYPOXIA          0xC002  /* SpO2 below threshold / stale  */
#define DTC_C0003_CNS_SIGNAL_LOSS  0xC003  /* brain heartbeat frame missing */

/* ---------------------------------------------------------------------
 * UDS-style service IDs implemented by each organ's diagnostic server
 * (subset of the real ISO 14229 service table)
 * ------------------------------------------------------------------- */

#define UDS_READ_DATA_BY_ID        0x22
#define UDS_READ_DTC_INFO          0x19
#define UDS_CLEAR_DTC              0x14

/* Negative response codes (subset) */
#define UDS_NRC_GENERAL_REJECT     0x10
#define UDS_NRC_REQUEST_OUT_OF_RANGE 0x31

/* Data identifiers (DIDs) used with ReadDataByIdentifier (0x22) */
#define DID_HEART_RATE      0xF010
#define DID_BLOOD_PRESSURE  0xF011
#define DID_SPO2            0xF020
#define DID_RESP_RATE       0xF021
#define DID_BRAIN_STATUS    0xF030

/* ---------------------------------------------------------------------
 * Fault-detection timing (network management style timeout)
 * ------------------------------------------------------------------- */

#define HEARTBEAT_PERIOD_MS     100
#define HEARTBEAT_TIMEOUT_MS    350   /* ~3 missed cycles = declare fault */

#endif /* CANIDS_H */

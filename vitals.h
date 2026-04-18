/*
 * vitals.h
 * Shared data contract between the MAX30102 Resource Manager
 * and any client application that opens /dev/vital_sensor.
 *
 * Both processes include this header — no tight coupling.
 */

#ifndef VITALS_H
#define VITALS_H

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Device path exposed by the Resource Manager                         */
/* ------------------------------------------------------------------ */
#define VITAL_SENSOR_PATH   "/dev/vital_sensor"

/* ------------------------------------------------------------------ */
/* Custom devctl() command codes                                        */
/* DCMD_VITAL_* : allows typed control without parsing text            */
/* ------------------------------------------------------------------ */
#include <devctl.h>

#define VITAL_CMD_CODE      1

/* Read the latest snapshot */
#define DCMD_VITAL_READ     __DIOF(_DCMD_MISC, VITAL_CMD_CODE + 0, vital_data_t)

/* Reset all accumulators (BPM history, SpO2 window) */
#define DCMD_VITAL_RESET    __DION(_DCMD_MISC, VITAL_CMD_CODE + 1)

/* ------------------------------------------------------------------ */
/* Vital-signs snapshot — written atomically by the driver             */
/* ------------------------------------------------------------------ */
typedef struct {
    float    bpm;               /* Heart rate in beats per minute      */
    float    spo2;              /* Blood-oxygen saturation 70–100 %    */
    int      finger_detected;   /* Non-zero when finger is on sensor   */
    uint32_t ir_raw;            /* Latest raw IR ADC count             */
    uint32_t red_raw;           /* Latest raw Red ADC count            */
    uint64_t timestamp_ms;      /* Monotonic ms at time of sample      */
} vital_data_t;

#endif /* VITALS_H */

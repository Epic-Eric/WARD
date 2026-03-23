#pragma once
#include <Arduino.h>

// ── Scan geometry ──────────────────────────────────────────────────
// Full yaw rotation, limited pitch window
#define PC_YAW_STEPS_FULL   200   // steps for full 360° yaw
#define PC_PITCH_STEPS       30   // total pitch range (centered)
#define PC_COARSE_YAW_SKIP    4   // coarse pass: step every N yaw steps
#define PC_COARSE_PITCH_SKIP   4  // coarse pass: step every N pitch steps
#define PC_FINE_RADIUS         2  // how many coarse cells around an edge to refine

// ── Measurement robustness ────────────────────────────────────────
#define PC_SAMPLES_COARSE      3  // readings per coarse point
#define PC_SAMPLES_FINE        5  // readings per fine point
#define PC_EDGE_THRESHOLD_MM 100  // distance jump (mm) that counts as an edge

// Entry point — call from main loop on serial command
void point_cloud_scan(int stepDelayUs);

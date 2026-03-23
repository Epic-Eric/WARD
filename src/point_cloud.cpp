#include "point_cloud.h"
#include <Arduino.h>
#include <VL53L1X.h>

// ── Externals (defined in main.cpp) ───────────────────────────────
extern VL53L1X sensor;
void   stepMotor(int stepPin, int dirPin, bool direction, int steps);
double get_distance();

// Pin definitions must match main.cpp
#define PITCH_DIR_PIN  2
#define PITCH_STEP_PIN 3
#define YAW_DIR_PIN    6
#define YAW_STEP_PIN   7

// ── Helpers ───────────────────────────────────────────────────────

// Insertion-sort a small array in-place
static void sort_doubles(double* arr, int n) {
    for (int i = 1; i < n; i++) {
        double key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// Take multiple readings, reject outliers via IQR, return median of inliers.
// Returns -1 only if every reading failed.
static double robust_distance(int numSamples) {
    const int MAX_SAMPLES = 9;
    if (numSamples > MAX_SAMPLES) numSamples = MAX_SAMPLES;

    double readings[MAX_SAMPLES];
    int valid = 0;

    for (int i = 0; i < numSamples; i++) {
        double d = get_distance();
        if (d >= 0) {
            readings[valid++] = d;
        }
    }

    if (valid == 0) return -1;
    if (valid == 1) return readings[0];

    sort_doubles(readings, valid);

    // IQR-based outlier removal (only meaningful with >= 4 points)
    if (valid >= 4) {
        double q1 = readings[valid / 4];
        double q3 = readings[(3 * valid) / 4];
        double iqr = q3 - q1;
        double lo = q1 - 1.5 * iqr;
        double hi = q3 + 1.5 * iqr;

        // Filter in-place
        int kept = 0;
        for (int i = 0; i < valid; i++) {
            if (readings[i] >= lo && readings[i] <= hi) {
                readings[kept++] = readings[i];
            }
        }
        if (kept > 0) valid = kept;
        // if all removed (shouldn't happen), keep originals
    }

    // Return median of remaining
    return readings[valid / 2];
}

// Move motor and track position delta (returns new position)
static void moveYaw(int steps, bool forward, int stepDelay) {
    stepMotor(YAW_STEP_PIN, YAW_DIR_PIN, forward, steps);
    (void)stepDelay; // delay is inside stepMotor
}

static void movePitch(int steps, bool forward, int stepDelay) {
    stepMotor(PITCH_STEP_PIN, PITCH_DIR_PIN, forward, steps);
    (void)stepDelay;
}

// Move pitch to an absolute step position from the current one
static void movePitchTo(int current, int target, int stepDelay) {
    if (target == current) return;
    bool forward = (target > current);
    int delta = abs(target - current);
    movePitch(delta, forward, stepDelay);
}

// ── Coarse scan storage ───────────────────────────────────────────
// We keep the coarse grid in memory to detect edges before the fine pass.
// Grid size: (yaw coarse count) x (pitch coarse count)

#define COARSE_YAW_COUNT  ((PC_YAW_STEPS_FULL  / PC_COARSE_YAW_SKIP) + 1)
#define COARSE_PITCH_COUNT ((PC_PITCH_STEPS / PC_COARSE_PITCH_SKIP) + 1)

// On an RP2040 with 264 KB RAM, this is fine (~2 KB for a 50x8 grid).
static double coarseGrid[COARSE_YAW_COUNT][COARSE_PITCH_COUNT];
static bool   edgeFlag[COARSE_YAW_COUNT][COARSE_PITCH_COUNT];

// ── Output one point ──────────────────────────────────────────────
static void emitPoint(int yawStep, int pitchStep, double dist) {
    Serial.print(yawStep);
    Serial.print(",");
    Serial.print(pitchStep);
    Serial.print(",");
    Serial.println(dist, 1);
}

// ── Main scan routine ─────────────────────────────────────────────
void point_cloud_scan(int stepDelayUs) {
    (void)stepDelayUs; // step delay baked into stepMotor via STEP_DELAY_US

    int coarseYawN  = COARSE_YAW_COUNT;
    int coarsePitchN = COARSE_PITCH_COUNT;

    Serial.println("PC_START");
    Serial.println("yaw,pitch,distance_mm");

    // ────────────────────────────────────────────────────────────
    // PASS 1 — Coarse serpentine scan
    // ────────────────────────────────────────────────────────────
    int curPitchStep = 0; // absolute pitch position in motor steps

    for (int yi = 0; yi < coarseYawN; yi++) {
        bool pitchForward = (yi % 2 == 0);

        for (int pi = 0; pi < coarsePitchN; pi++) {
            int pitchIdx = pitchForward ? pi : (coarsePitchN - 1 - pi);
            int targetPitch = pitchIdx * PC_COARSE_PITCH_SKIP;

            movePitchTo(curPitchStep, targetPitch, stepDelayUs);
            curPitchStep = targetPitch;

            double dist = robust_distance(PC_SAMPLES_COARSE);
            int yawAbsolute = yi * PC_COARSE_YAW_SKIP;

            coarseGrid[yi][pitchIdx] = dist;
            edgeFlag[yi][pitchIdx] = false;

            emitPoint(yawAbsolute, targetPitch, dist);
        }

        // Step yaw to next coarse column
        if (yi < coarseYawN - 1) {
            moveYaw(PC_COARSE_YAW_SKIP, true, stepDelayUs);
        }
    }

    // ────────────────────────────────────────────────────────────
    // Edge detection on coarse grid
    // ────────────────────────────────────────────────────────────
    int edgeCount = 0;
    for (int yi = 0; yi < coarseYawN; yi++) {
        for (int pi = 0; pi < coarsePitchN; pi++) {
            double d = coarseGrid[yi][pi];
            if (d < 0) continue;

            // Check 4-connected neighbours
            bool isEdge = false;
            int dy[] = {-1, 1, 0, 0};
            int dp[] = {0, 0, -1, 1};
            for (int k = 0; k < 4; k++) {
                int ny = yi + dy[k];
                int np = pi + dp[k];
                if (ny < 0 || ny >= coarseYawN || np < 0 || np >= coarsePitchN)
                    continue;
                double nd = coarseGrid[ny][np];
                if (nd < 0) continue;
                if (abs(d - nd) > PC_EDGE_THRESHOLD_MM) {
                    isEdge = true;
                    break;
                }
            }

            if (isEdge) {
                // Flag this cell and its neighbours within FINE_RADIUS
                for (int ry = -PC_FINE_RADIUS; ry <= PC_FINE_RADIUS; ry++) {
                    for (int rp = -PC_FINE_RADIUS; rp <= PC_FINE_RADIUS; rp++) {
                        int ey = yi + ry;
                        int ep = pi + rp;
                        if (ey >= 0 && ey < coarseYawN && ep >= 0 && ep < coarsePitchN) {
                            if (!edgeFlag[ey][ep]) {
                                edgeFlag[ey][ep] = true;
                                edgeCount++;
                            }
                        }
                    }
                }
            }
        }
    }

    Serial.print("# edges_to_refine: ");
    Serial.println(edgeCount);

    // ────────────────────────────────────────────────────────────
    // PASS 2 — Fine scan around detected edges
    // ────────────────────────────────────────────────────────────
    if (edgeCount > 0) {
        // Return to yaw=0 first
        int curYawStep = (coarseYawN - 1) * PC_COARSE_YAW_SKIP;
        moveYaw(curYawStep, false, stepDelayUs); // go back to yaw 0
        curYawStep = 0;
        // Return pitch to 0
        movePitchTo(curPitchStep, 0, stepDelayUs);
        curPitchStep = 0;

        for (int yi = 0; yi < coarseYawN; yi++) {
            bool hasEdgeInRow = false;
            for (int pi = 0; pi < coarsePitchN; pi++) {
                if (edgeFlag[yi][pi]) { hasEdgeInRow = true; break; }
            }
            if (!hasEdgeInRow) {
                // Skip this yaw row entirely
                continue;
            }

            // Move yaw to this coarse column
            int targetYaw = yi * PC_COARSE_YAW_SKIP;
            if (targetYaw != curYawStep) {
                int delta = targetYaw - curYawStep;
                moveYaw(abs(delta), delta > 0, stepDelayUs);
                curYawStep = targetYaw;
            }

            for (int pi = 0; pi < coarsePitchN; pi++) {
                if (!edgeFlag[yi][pi]) continue;

                // Fine-scan within this coarse cell
                int pitchBase = pi * PC_COARSE_PITCH_SKIP;
                int yawBase   = yi * PC_COARSE_YAW_SKIP;

                // Scan each single step within the coarse cell
                for (int fy = 0; fy < PC_COARSE_YAW_SKIP; fy++) {
                    int yawAbs = yawBase + fy;
                    if (yawAbs >= PC_YAW_STEPS_FULL) break;

                    // Move yaw to exact position
                    if (yawAbs != curYawStep) {
                        int delta = yawAbs - curYawStep;
                        moveYaw(abs(delta), delta > 0, stepDelayUs);
                        curYawStep = yawAbs;
                    }

                    for (int fp = 0; fp < PC_COARSE_PITCH_SKIP; fp++) {
                        int pitchAbs = pitchBase + fp;
                        if (pitchAbs >= PC_PITCH_STEPS) break;

                        // Skip if this is already a coarse-grid point
                        if (fy == 0 && fp == 0) continue;

                        movePitchTo(curPitchStep, pitchAbs, stepDelayUs);
                        curPitchStep = pitchAbs;

                        double dist = robust_distance(PC_SAMPLES_FINE);
                        emitPoint(yawAbs, pitchAbs, dist);
                    }
                }
            }
        }

        // Return to home
        moveYaw(curYawStep, false, stepDelayUs);
        movePitchTo(curPitchStep, 0, stepDelayUs);
    } else {
        // No edges — return to home
        int curYawStep = (coarseYawN - 1) * PC_COARSE_YAW_SKIP;
        moveYaw(curYawStep, false, stepDelayUs);
        movePitchTo(curPitchStep, 0, stepDelayUs);
    }

    Serial.println("PC_DONE");
}

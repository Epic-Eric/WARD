#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// Pin definitions
#define PITCH_DIR_PIN  2
#define PITCH_STEP_PIN 3
#define YAW_DIR_PIN    6
#define YAW_STEP_PIN   7

// Tuning
#define STEP_DELAY_US  100   // microseconds between step pulses (lower = faster)
#define STEPS_PER_KEY  10    // steps per keypress

// Sweep parameters
#define MAX_YAW_STEPS   20
#define MAX_PITCH_STEPS 20

// Nano RP2040 Connect I2C pins
#define SDA_PIN 18  // A4
#define SCL_PIN 19  // A5
MbedI2C MyWire(SDA_PIN, SCL_PIN);

VL53L1X sensor;

// Forward declarations
void stepMotor(int stepPin, int dirPin, bool direction, int steps);
double get_distance();
void sweep(int maxYawSteps, int maxPitchSteps);

double get_distance() {
    // Wait for data to become ready (blocking, with timeout)
    unsigned long start = millis();
    while (!sensor.dataReady()) {
        if (millis() - start > 500) {
            return -1; // timeout
        }
    }

    uint16_t distance = sensor.read(false);

    if (sensor.ranging_data.range_status == VL53L1X::RangeValid) {
        return (double)distance;
    }
    return -1; // invalid reading
}

void stepMotor(int stepPin, int dirPin, bool direction, int steps) {
    digitalWrite(dirPin, direction ? HIGH : LOW);
    for (int i = 0; i < steps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(STEP_DELAY_US);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(STEP_DELAY_US);
    }
}

void sweep(int maxYawSteps, int maxPitchSteps) {
    // Serpentine scan: for each yaw step, sweep pitch forward then backward
    // Prints CSV over Serial: yawStep, pitchStep, distance_mm
    Serial.println("yaw,pitch,distance_mm");

    for (int yawStep = 0; yawStep < maxYawSteps; yawStep++) {
        bool pitchForward = (yawStep % 2 == 0);

        for (int pitchStep = 0; pitchStep < maxPitchSteps; pitchStep++) {
            double dist = get_distance();

            int pitchIndex = pitchForward ? pitchStep : (maxPitchSteps - 1 - pitchStep);
            Serial.print(yawStep);
            Serial.print(",");
            Serial.print(pitchIndex);
            Serial.print(",");
            Serial.println(dist);

            if (pitchStep < maxPitchSteps - 1) {
                stepMotor(PITCH_STEP_PIN, PITCH_DIR_PIN, pitchForward, 1);
            }
        }

        if (yawStep < maxYawSteps - 1) {
            stepMotor(YAW_STEP_PIN, YAW_DIR_PIN, true, 1);
        }
    }

    Serial.println("SWEEP_DONE");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) { delay(10); }

    pinMode(PITCH_DIR_PIN, OUTPUT);
    pinMode(PITCH_STEP_PIN, OUTPUT);
    pinMode(YAW_DIR_PIN, OUTPUT);
    pinMode(YAW_STEP_PIN, OUTPUT);

    Serial.println("WASD Stepper Control Ready");
    Serial.println("W=pitch up  S=pitch down  A=yaw left  D=yaw right");
    Serial.println("G=start sweep");

    MyWire.begin();
    MyWire.setClock(400000);

    sensor.setBus(&MyWire);
    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.println("ERROR: Failed to detect VL53L1X!");
        Serial.println("Check wiring:");
        Serial.println("  A4 -> SDA");
        Serial.println("  A5 -> SCL");
        Serial.println("  3.3V -> VIN");
        Serial.println("  GND  -> GND");
        while (1) { delay(10); }
    }

    Serial.println("Sensor initialized OK");

    sensor.setDistanceMode(VL53L1X::Long);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(50);

    Serial.println("Ready.\n");
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();

        switch (c) {
            case 'w':
                stepMotor(PITCH_STEP_PIN, PITCH_DIR_PIN, HIGH, STEPS_PER_KEY);
                break;
            case 's':
                stepMotor(PITCH_STEP_PIN, PITCH_DIR_PIN, LOW, STEPS_PER_KEY);
                break;
            case 'a':
                stepMotor(YAW_STEP_PIN, YAW_DIR_PIN, HIGH, STEPS_PER_KEY);
                break;
            case 'd':
                stepMotor(YAW_STEP_PIN, YAW_DIR_PIN, LOW, STEPS_PER_KEY);
                break;
            case 'g':
                sweep(MAX_YAW_STEPS, MAX_PITCH_STEPS);
                break;
        }
    }
}

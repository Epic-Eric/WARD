// #include <Arduino.h>
// #include <Wire.h>
// #include <VL53L1X.h>

// // Nano RP2040 Connect I2C pins
// #define SDA_PIN 11  // A4
// #define SCL_PIN 10  // A5
// // sccl on 7, sda on 6
// MbedI2C MyWire(SDA_PIN, SCL_PIN);

// VL53L1X sensor;

// void setup() {
//     Serial.begin(115200);
//     delay(2000);

//     MyWire.begin();
//     MyWire.setClock(400000);

//     Serial.println("VL53L1X Test");
//     Serial.println("------------");

//     sensor.setBus(&MyWire);
//     sensor.setTimeout(500);

//     if (!sensor.init()) {
//         Serial.println("ERROR: Failed to detect VL53L1X!");
//         Serial.println("Check wiring:");
//         Serial.println("  A4 -> SDA");
//         Serial.println("  A5 -> SCL");
//         Serial.println("  3.3V -> VIN");
//         Serial.println("  GND  -> GND");
//         while (1) { delay(10); }
//     }

//     Serial.println("Sensor initialized OK");

//     sensor.setDistanceMode(VL53L1X::Long);
//     sensor.setMeasurementTimingBudget(50000);
//     sensor.startContinuous(50);

//     Serial.println("Reading distances...\n");
// }

// void loop() {
//     if (sensor.dataReady()) {
//         uint16_t distance = sensor.read(false);

//         if (sensor.ranging_data.range_status == VL53L1X::RangeValid) {
//             Serial.print("Distance: ");
//             Serial.print(distance);
//             Serial.print(" mm  (");
//             Serial.print(distance / 10.0, 1);
//             Serial.println(" cm)");
//         } else {
//             Serial.print("Range error: ");
//             Serial.println(sensor.rangeStatusToString(sensor.ranging_data.range_status));
//         }
//     }
//     delay(1);
// }

#include <Arduino.h>
#include <Stepper.h>

// Steps per revolution for your motors (adjust to your motor's spec)
#define STEPS_PER_REV 200

// DRV8833 #1 — Pitch motor
#define PITCH_PIN1  17   // BIN1
#define PITCH_PIN2  16   // BIN2
#define PITCH_PIN3  15   // AIN2
#define PITCH_PIN4  14   // AIN1

#define YAW_PIN1    6
#define YAW_PIN2    7
#define YAW_PIN3   8
#define YAW_PIN4   9

// Tuning
#define MOTOR_SPEED   10  // RPM
#define STEPS_PER_KEY 1  // steps per keypress

Stepper yawMotor(STEPS_PER_REV, YAW_PIN1, YAW_PIN2, YAW_PIN3, YAW_PIN4);
Stepper pitchMotor(STEPS_PER_REV, PITCH_PIN1, PITCH_PIN2, PITCH_PIN3, PITCH_PIN4);

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pitchMotor.setSpeed(MOTOR_SPEED);
  yawMotor.setSpeed(MOTOR_SPEED);

  Serial.println("WASD Stepper Control (DRV8833) Ready");
  Serial.println("W=pitch up  S=pitch down  A=yaw left  D=yaw right");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'w':
        pitchMotor.step(STEPS_PER_KEY);
        break;
      case 's':
        pitchMotor.step(-STEPS_PER_KEY);
        break;
      case 'a':
        yawMotor.step(-STEPS_PER_KEY);
        break;
      case 'd':
        yawMotor.step(STEPS_PER_KEY);
        break;
    }
  }
}
// #include <Arduino.h>
// #include <Wire.h>
// #include <VL53L1X.h>

// // Nano RP2040 Connect I2C pins
// #define SDA_PIN 8  // A4
// #define SCL_PIN 9  // A5
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

// Pin definitions
#define PITCH_DIR_PIN  2
#define PITCH_STEP_PIN 3
#define YAW_DIR_PIN    6
#define YAW_STEP_PIN   7

// Tuning
#define STEP_DELAY_US  100   // microseconds between step pulses (lower = faster)
#define STEPS_PER_KEY  10    // steps per keypress

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  pinMode(PITCH_DIR_PIN, OUTPUT);
  pinMode(PITCH_STEP_PIN, OUTPUT);
  pinMode(YAW_DIR_PIN, OUTPUT);
  pinMode(YAW_STEP_PIN, OUTPUT);

  Serial.println("WASD Stepper Control Ready");
  Serial.println("W=pitch up  S=pitch down  A=yaw left  D=yaw right");
  Serial.println("Hold keys for continuous movement.");
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
    }
  }
}
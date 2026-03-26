#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <Stepper.h>
#include <WiFiNINA.h>

namespace Config {
constexpr unsigned long kSerialBaudRate = 115200;

constexpr char kWifiSsid[] = "S";
constexpr char kWifiPassword[] = "87654321";
constexpr char kServerHost[] = "Haysons-MacBook-Pro.local";
const IPAddress kServerFallbackIp(172, 20, 10, 4);
constexpr uint16_t kServerPort = 9000;

constexpr unsigned long kWifiRetryDelayMs = 1500;
constexpr unsigned long kServerRetryDelayMs = 1000;
constexpr unsigned long kSampleSettleDelayMs = 80;
constexpr unsigned long kSensorWaitTimeoutMs = 120;
constexpr unsigned long kFramePauseMs = 300;
constexpr unsigned long kClientCloseDelayMs = 100;

constexpr int kSdaPin = 8;
constexpr int kSclPin = 9;

constexpr int kPitchPin1 = 17;
constexpr int kPitchPin2 = 16;
constexpr int kPitchPin3 = 15;
constexpr int kPitchPin4 = 14;
constexpr int kYawPin1 = 5;
constexpr int kYawPin2 = 4;
constexpr int kYawPin3 = 3;
constexpr int kYawPin4 = 2;

constexpr int kStepsPerRevolution = 200;
constexpr float kYawGearRatio = 12.0f;
constexpr float kPitchGearRatio = 2.0f;
constexpr float kYawAnglePerStepDeg =
    360.0f / static_cast<float>(kStepsPerRevolution) / kYawGearRatio;
constexpr float kPitchAnglePerStepDeg =
    360.0f / static_cast<float>(kStepsPerRevolution) / kPitchGearRatio;

constexpr float kPitchStartingAngleDeg = -60.0f;
constexpr float kYawStartingAngleDeg = 0.0f;

constexpr int kYawMotorSpeedRpm = 20;
constexpr int kPitchMotorSpeedRpm = 20;

constexpr float kPitchSweepStartDeg = -30.0f;
constexpr float kPitchSweepEndDeg = 0.0f;
constexpr float kYawSweepStartDeg = 0.0f;
constexpr float kYawSweepEndDeg = 20.0f;
constexpr float kYawSweepStepDeg = 1.0f;
}  // namespace Config

struct Point3D {
    float x;
    float y;
    float z;
};

struct ScanPoint {
    uint32_t frameId;
    uint16_t pointIndex;
    float yawDeg;
    float pitchDeg;
    uint16_t distanceMm;
    Point3D positionMm;
};

Point3D sphericalToCartesian(float radiusMm, float yawDeg, float pitchDeg) {
    const float yawRad = yawDeg * DEG_TO_RAD;
    const float pitchRad = pitchDeg * DEG_TO_RAD;
    const float cosPitch = cos(pitchRad);

    Point3D point{};
    point.x = radiusMm * cosPitch * cos(yawRad);
    point.y = radiusMm * cosPitch * sin(yawRad);
    point.z = radiusMm * sin(pitchRad);
    return point;
}

class StepperAxis {
public:
    StepperAxis(int pin1, int pin2, int pin3, int pin4, float anglePerStepDeg,
                float initialAngleDeg, float homeAngleDeg)
        : motor_(Config::kStepsPerRevolution, pin1, pin2, pin3, pin4),
          anglePerStepDeg_(anglePerStepDeg),
          currentAngleDeg_(initialAngleDeg),
          homeAngleDeg_(homeAngleDeg) {}

    void setSpeedRpm(int rpm) { motor_.setSpeed(rpm); }

    float currentAngleDeg() const { return currentAngleDeg_; }

    void moveTo(float targetAngleDeg) {
        const float deltaDeg = targetAngleDeg - currentAngleDeg_;
        const int steps = lroundf(deltaDeg / anglePerStepDeg_);
        if (steps == 0) {
            return;
        }

        motor_.step(steps);
        currentAngleDeg_ += static_cast<float>(steps) * anglePerStepDeg_;
    }

    void zero() { moveTo(0.0f); }

    void toStartingPosition() { moveTo(homeAngleDeg_); }

private:
    Stepper motor_;
    float anglePerStepDeg_;
    float currentAngleDeg_;
    float homeAngleDeg_;
};

class DistanceSensor {
public:
    explicit DistanceSensor(MbedI2C& bus) : bus_(bus) {}

    bool begin() {
        sensor_.setBus(&bus_);
        sensor_.setTimeout(500);

        if (!sensor_.init()) {
            Serial.println("ERROR: Failed to detect VL53L1X.");
            Serial.println("Check wiring and power.");
            return false;
        }

        sensor_.setDistanceMode(VL53L1X::Long);
        sensor_.setMeasurementTimingBudget(50000);
        sensor_.startContinuous(50);
        return true;
    }

    bool readDistanceMm(uint16_t& distanceMm, const char*& errorText) {
        const unsigned long startMs = millis();
        while (!sensor_.dataReady()) {
            if (millis() - startMs >= Config::kSensorWaitTimeoutMs) {
                errorText = "SENSOR_TIMEOUT";
                return false;
            }
            delay(1);
        }

        distanceMm = sensor_.read(false);
        if (sensor_.ranging_data.range_status != VL53L1X::RangeValid) {
            errorText = sensor_.rangeStatusToString(sensor_.ranging_data.range_status);
            return false;
        }

        errorText = nullptr;
        return true;
    }

private:
    VL53L1X sensor_;
    MbedI2C& bus_;
};

class PointCloudStreamer {
public:
    PointCloudStreamer(const char* ssid, const char* password, const char* serverHost,
                       IPAddress serverFallbackIp, uint16_t serverPort)
        : ssid_(ssid),
          password_(password),
          serverHost_(serverHost),
          serverFallbackIp_(serverFallbackIp),
          serverPort_(serverPort),
          lastServerConnectAttemptMs_(0) {}

    bool begin() {
        if (WiFi.status() == WL_NO_MODULE) {
            Serial.println("ERROR: WiFi module not detected.");
            return false;
        }

        connectWifiBlocking();
        ensureServerConnection();
        return true;
    }

    bool beginFrame(uint32_t frameId) {
        if (!ensureServerConnection()) {
            Serial.println("WARN: Server unavailable, frame will only be logged on Serial.");
            return false;
        }

        client_.print("RESET,");
        client_.println(frameId);

        client_.print("FRAME_BEGIN,");
        client_.print(frameId);
        client_.print(",");
        client_.println(millis());
        return true;
    }

    void sendPoint(const ScanPoint& point, bool networkEnabled) {
        Serial.print("POINT,");
        Serial.print(point.frameId);
        Serial.print(",");
        Serial.print(point.pointIndex);
        Serial.print(",");
        Serial.print(point.yawDeg, 2);
        Serial.print(",");
        Serial.print(point.pitchDeg, 2);
        Serial.print(",");
        Serial.print(point.distanceMm);
        Serial.print(",");
        Serial.print(point.positionMm.x, 2);
        Serial.print(",");
        Serial.print(point.positionMm.y, 2);
        Serial.print(",");
        Serial.println(point.positionMm.z, 2);

        if (!networkEnabled || !client_.connected()) {
            return;
        }

        client_.print("POINT,");
        client_.print(point.frameId);
        client_.print(",");
        client_.print(point.pointIndex);
        client_.print(",");
        client_.print(point.yawDeg, 2);
        client_.print(",");
        client_.print(point.pitchDeg, 2);
        client_.print(",");
        client_.print(point.distanceMm);
        client_.print(",");
        client_.print(point.positionMm.x, 2);
        client_.print(",");
        client_.print(point.positionMm.y, 2);
        client_.print(",");
        client_.println(point.positionMm.z, 2);
    }

    void endFrame(uint32_t frameId, uint16_t pointCount, bool networkEnabled) {
        Serial.print("FRAME_END,");
        Serial.print(frameId);
        Serial.print(",");
        Serial.println(pointCount);

        if (!networkEnabled || !client_.connected()) {
            return;
        }

        client_.print("FRAME_END,");
        client_.print(frameId);
        client_.print(",");
        client_.println(pointCount);
    }

    void close() {
        if (!client_.connected()) {
            return;
        }

        client_.flush();
        delay(Config::kClientCloseDelayMs);
        client_.stop();
        Serial.println("Server connection closed.");
    }

private:
    void connectWifiBlocking() {
        Serial.println("Connecting to WiFi... Looking for available networks...");
        int n = WiFi.scanNetworks();
        // List available networks for debugging, but don't fail if the target SSID isn't found since it could be hidden.
        Serial.println("Available WiFi networks:");
        for (int i = 0; i < n; i++) {
             Serial.println(WiFi.SSID(i));
        }

        while (WiFi.status() != WL_CONNECTED) {
            Serial.print("Connecting to WiFi SSID ");
            Serial.println(ssid_);

            const int status = WiFi.begin(ssid_, password_);
            if (status == WL_CONNECTED) {
                break;
            }

            Serial.print("WiFi connection failed, status=");
            Serial.println(status);
            delay(Config::kWifiRetryDelayMs);
        }

        Serial.print("WiFi connected. Board IP: ");
        Serial.println(WiFi.localIP());
    }

    bool ensureServerConnection() {
        if (WiFi.status() != WL_CONNECTED) {
            connectWifiBlocking();
        }

        if (client_.connected()) {
            return true;
        }

        const unsigned long nowMs = millis();
        if (nowMs - lastServerConnectAttemptMs_ < Config::kServerRetryDelayMs) {
            return false;
        }
        lastServerConnectAttemptMs_ = nowMs;

        client_.stop();

        IPAddress resolvedIp;
        if (WiFi.hostByName(serverHost_, resolvedIp) == 1) {
            Serial.print("Resolved server host ");
            Serial.print(serverHost_);
            Serial.print(" to ");
            Serial.println(resolvedIp);

            if (client_.connect(resolvedIp, serverPort_)) {
                client_.println("HELLO,NANO_RP2040_CONNECT,POINT_CLOUD_V1");
                Serial.println("Server connected.");
                return true;
            }

            Serial.println("Resolved host connect failed.");
        } else {
            Serial.print("Host lookup failed for ");
            Serial.println(serverHost_);
        }

        Serial.print("Falling back to server IP ");
        Serial.print(serverFallbackIp_);
        Serial.print(":");
        Serial.println(serverPort_);

        if (!client_.connect(serverFallbackIp_, serverPort_)) {
            Serial.println("Server connection failed.");
            return false;
        }

        client_.println("HELLO,NANO_RP2040_CONNECT,POINT_CLOUD_V1");
        Serial.println("Server connected.");
        return true;
    }

    const char* ssid_;
    const char* password_;
    const char* serverHost_;
    IPAddress serverFallbackIp_;
    uint16_t serverPort_;
    unsigned long lastServerConnectAttemptMs_;
    WiFiClient client_;
};

class PointCloudScanner {
public:
    PointCloudScanner(StepperAxis& yawAxis, StepperAxis& pitchAxis,
                      DistanceSensor& distanceSensor, PointCloudStreamer& streamer)
        : yawAxis_(yawAxis),
          pitchAxis_(pitchAxis),
          distanceSensor_(distanceSensor),
          streamer_(streamer),
          nextFrameId_(1) {}

    void begin() {
        yawAxis_.setSpeedRpm(Config::kYawMotorSpeedRpm);
        pitchAxis_.setSpeedRpm(Config::kPitchMotorSpeedRpm);

        pitchAxis_.moveTo(0.0f);
        yawAxis_.moveTo(0.0f);
    }

    void runFrame() {
        const uint32_t frameId = nextFrameId_++;
        uint16_t pointIndex = 0;

        Serial.print("Starting frame ");
        Serial.println(frameId);

        const bool networkEnabled = streamer_.beginFrame(frameId);

        for (float yawTarget = Config::kYawSweepStartDeg;
             yawTarget <= Config::kYawSweepEndDeg + 0.001f;
             yawTarget += Config::kYawSweepStepDeg) {
            yawAxis_.moveTo(yawTarget);

            for (float pitchTarget = Config::kPitchSweepStartDeg;
                 pitchTarget <= Config::kPitchSweepEndDeg + 0.001f;
                 pitchTarget += Config::kPitchAnglePerStepDeg) {
                pitchAxis_.moveTo(pitchTarget);
                delay(Config::kSampleSettleDelayMs);

                uint16_t distanceMm = 0;
                const char* errorText = nullptr;
                if (!distanceSensor_.readDistanceMm(distanceMm, errorText)) {
                    Serial.print("Skipping point at yaw=");
                    Serial.print(yawAxis_.currentAngleDeg(), 2);
                    Serial.print(" pitch=");
                    Serial.print(pitchAxis_.currentAngleDeg(), 2);
                    Serial.print(" reason=");
                    Serial.println(errorText);
                    continue;
                }

                ScanPoint point{};
                point.frameId = frameId;
                point.pointIndex = pointIndex++;
                point.yawDeg = yawAxis_.currentAngleDeg();
                point.pitchDeg = pitchAxis_.currentAngleDeg();
                point.distanceMm = distanceMm;
                point.positionMm =
                    sphericalToCartesian(static_cast<float>(distanceMm), point.yawDeg,
                                         point.pitchDeg);

                streamer_.sendPoint(point, networkEnabled);
            }
        }

        streamer_.endFrame(frameId, pointIndex, networkEnabled);

        pitchAxis_.moveTo(0.0f);
        yawAxis_.moveTo(0.0f);
        delay(Config::kFramePauseMs);
    }

private:
    StepperAxis& yawAxis_;
    StepperAxis& pitchAxis_;
    DistanceSensor& distanceSensor_;
    PointCloudStreamer& streamer_;
    uint32_t nextFrameId_;
};

MbedI2C kI2cBus(Config::kSdaPin, Config::kSclPin);
StepperAxis kYawAxis(Config::kYawPin1, Config::kYawPin2, Config::kYawPin3,
                     Config::kYawPin4, Config::kYawAnglePerStepDeg,
                     Config::kYawStartingAngleDeg, Config::kYawStartingAngleDeg);
StepperAxis kPitchAxis(Config::kPitchPin1, Config::kPitchPin2, Config::kPitchPin3,
                       Config::kPitchPin4, Config::kPitchAnglePerStepDeg,
                       Config::kPitchStartingAngleDeg, Config::kPitchStartingAngleDeg);
DistanceSensor kDistanceSensor(kI2cBus);
PointCloudStreamer kStreamer(Config::kWifiSsid, Config::kWifiPassword,
                             Config::kServerHost, Config::kServerFallbackIp,
                             Config::kServerPort);
PointCloudScanner kScanner(kYawAxis, kPitchAxis, kDistanceSensor, kStreamer);
bool gScanEnded = false;

void setup() {
    Serial.begin(Config::kSerialBaudRate);
    delay(2000);

    kI2cBus.begin();
    kI2cBus.setClock(400000);

    if (!kDistanceSensor.begin()) {
        while (true) {
            delay(10);
        }
    }

    if (!kStreamer.begin()) {
        while (true) {
            delay(10);
        }
    }

    kScanner.begin();
    Serial.println("Point cloud scanner ready.");
}

void resetToStartingPosition() {
    kPitchAxis.toStartingPosition();
    kYawAxis.toStartingPosition();
}

void loop() {
    if (gScanEnded) {
        while (true) {
            delay(1000);
        }
    }

    kScanner.runFrame();
    kStreamer.close();
    gScanEnded = true;
    resetToStartingPosition();

    Serial.println("Scan complete. Scanner stopped.");
}

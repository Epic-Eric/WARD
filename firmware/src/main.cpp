#include <Arduino.h>
#include <Stepper.h>
#include <VL53L1X.h>
#include <WiFiNINA.h>
#include <Wire.h>

#include <cstring>

namespace Config {
constexpr unsigned long kSerialBaudRate = 115200;

constexpr char kWifiSsid[] = WIFI_SSID;
constexpr char kWifiPassword[] = WIFI_PASSWORD;
constexpr char kServerHost[] = SERVER_HOST;
const IPAddress kServerFallbackIp(SERVER_FALLBACK_IP);
constexpr uint16_t kServerPort = 9000;

constexpr unsigned long kWifiRetryDelayMs = 1500;
constexpr unsigned long kServerRetryDelayMs = 1000;
constexpr unsigned long kSampleSettleDelayMs = 80;
constexpr unsigned long kSensorWaitTimeoutMs = 120;
constexpr unsigned long kIdlePollDelayMs = 20;
constexpr unsigned long kHeartbeatIntervalMs = 1000;
constexpr unsigned long kSensorRecoverySettleDelayMs = 60;
constexpr uint8_t kSensorRecoveryAttempts = 4;
constexpr uint8_t kSensorRecoveryProbeAttempts = 2;
constexpr unsigned long kSensorRecoveryBackoffBaseMs = 75;

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
constexpr float kYawGearRatio = 6.0f;
constexpr float kPitchGearRatio = 1.0f;
constexpr float kYawAnglePerStepDeg =
    360.0f / static_cast<float>(kStepsPerRevolution) / kYawGearRatio;
constexpr float kPitchAnglePerStepDeg =
    360.0f / static_cast<float>(kStepsPerRevolution) / kPitchGearRatio;

constexpr float kPitchStartingAngleDeg = -120.0f;
constexpr float kYawStartingAngleDeg = 0.0f;

constexpr int kYawMotorSpeedRpm = 20;
constexpr int kPitchMotorSpeedRpm = 20;

constexpr float kPitchSweepStartDeg = -65.0f;
constexpr float kPitchSweepEndDeg = -25.0f;
constexpr float kYawSweepStartDeg = 0.0f;
constexpr float kDefaultScanDegrees = 20.0f;
constexpr float kMaxScanDegrees = 360.0f;
constexpr float kYawSweepStepDeg = 1.0f;

constexpr size_t kCommandBufferSize = 48;

// Sensor mounting geometry (relative to pitch axis intersection)
constexpr float kSensorAboveAxisMm = 35.0f;   // sensor is 35 mm above the pitch axis
constexpr float kSensorFaceOffsetMm = 34.0f;  // sensor face is 34 mm forward of the axis
}  // namespace Config

enum class RobotCommand {
    None,
    StartScan,
    StopScan,
    HardStop,
    ReleaseMotors,
    ZeroTurret,
    MoveTo,
};

enum class ScanOutcome {
    Completed,
    Stopped,
    HardStopped,
    HardStoppedReleased,
    SensorRecoveryFailed,
};

struct Point3D {
    float x;
    float y;
    float z;
};

Point3D sphericalToCartesian(float radiusMm, float yawDeg, float pitchDeg);

float axisDistanceFromSensorReading(float rawMm) {
    return rawMm + Config::kSensorFaceOffsetMm;
}

float pitchFromSensorReading(float axisDistanceMm, float motorPitchDeg) {
    if (axisDistanceMm <= 0.0f) {
        return motorPitchDeg;
    }
    const float parallaxCorrectionDeg =
        atanf(Config::kSensorAboveAxisMm / axisDistanceMm) / DEG_TO_RAD;
    return motorPitchDeg + parallaxCorrectionDeg;
}

Point3D scanPointFromSensorReading(float axisDistanceMm, float yawDeg, float pitchDeg) {
    return sphericalToCartesian(axisDistanceMm, yawDeg, pitchDeg);
}

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
          pin1_(pin1),
          pin2_(pin2),
          pin3_(pin3),
          pin4_(pin4),
          anglePerStepDeg_(anglePerStepDeg),
          currentAngleDeg_(initialAngleDeg),
          homeAngleDeg_(homeAngleDeg) {}

    void setSpeedRpm(int rpm) { motor_.setSpeed(rpm); }

    float currentAngleDeg() const { return currentAngleDeg_; }

    void setCurrentAngleDeg(float angleDeg) { currentAngleDeg_ = angleDeg; }

    void zero() { currentAngleDeg_ = 0.0f; }

    void moveTo(float targetAngleDeg) {
        const float deltaDeg = targetAngleDeg - currentAngleDeg_;
        const int steps = lroundf(deltaDeg / anglePerStepDeg_);
        if (steps == 0) {
            return;
        }

        motor_.step(steps);
        currentAngleDeg_ += static_cast<float>(steps) * anglePerStepDeg_;
    }

    void toStartingPosition() { moveTo(homeAngleDeg_); }

    void release() {
        digitalWrite(pin1_, LOW);
        digitalWrite(pin2_, LOW);
        digitalWrite(pin3_, LOW);
        digitalWrite(pin4_, LOW);
    }

private:
    Stepper motor_;
    int pin1_;
    int pin2_;
    int pin3_;
    int pin4_;
    float anglePerStepDeg_;
    float currentAngleDeg_;
    float homeAngleDeg_;
};

class DistanceSensor {
public:
    explicit DistanceSensor(MbedI2C& bus) : bus_(bus) {}

    bool begin() { return initializeSensor(true); }

    bool reconnect(const char*& errorText) {
        for (uint8_t attempt = 0; attempt < Config::kSensorRecoveryAttempts; ++attempt) {
            const unsigned long backoffMs =
                Config::kSensorRecoveryBackoffBaseMs << attempt;
            stopContinuousQuietly();
            delay(Config::kSensorRecoverySettleDelayMs);
            if (!initializeSensor(false)) {
                errorText = "SENSOR_INIT_FAILED";
                delay(backoffMs);
                continue;
            }

            for (uint8_t probe = 0; probe < Config::kSensorRecoveryProbeAttempts; ++probe) {
                uint16_t ignoredDistance = 0;
                const char* probeError = nullptr;
                if (readDistanceMm(ignoredDistance, probeError)) {
                    errorText = nullptr;
                    return true;
                }

                errorText = probeError;
                delay(Config::kSensorRecoverySettleDelayMs);
            }

            delay(backoffMs);
        }

        if (errorText == nullptr) {
            errorText = "SENSOR_RECOVERY_FAILED";
        }
        return false;
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
    bool initializeSensor(bool logErrors) {
        bus_.begin();
        bus_.setClock(400000);
        sensor_.setBus(&bus_);
        sensor_.setTimeout(500);

        if (!sensor_.init()) {
            if (logErrors) {
                Serial.println("ERROR: Failed to detect VL53L1X.");
                Serial.println("Check wiring and power.");
            }
            return false;
        }

        sensor_.setDistanceMode(VL53L1X::Long);
        sensor_.setMeasurementTimingBudget(50000);
        sensor_.startContinuous(50);
        return true;
    }

    void stopContinuousQuietly() {
        sensor_.stopContinuous();
    }

    VL53L1X sensor_;
    MbedI2C& bus_;
};

class RobotLink {
public:
    RobotLink(const char* ssid, const char* password, const char* serverHost,
              IPAddress serverFallbackIp, uint16_t serverPort)
        : ssid_(ssid),
          password_(password),
          serverHost_(serverHost),
          serverFallbackIp_(serverFallbackIp),
          serverPort_(serverPort),
          lastServerConnectAttemptMs_(0),
          lastHeartbeatSentMs_(0),
          requestedScanDegrees_(Config::kDefaultScanDegrees),
          requestedMoveYawDeg_(0.0f),
          requestedMovePitchDeg_(0.0f),
          commandLength_(0) {
        commandBuffer_[0] = '\0';
    }

    bool begin() {
        if (WiFi.status() == WL_NO_MODULE) {
            Serial.println("ERROR: WiFi module not detected.");
            return false;
        }

        connectWifiBlocking();
        ensureServerConnection();
        return true;
    }

    RobotCommand pollCommand() {
        if (!ensureServerConnection()) {
            return RobotCommand::None;
        }

        while (client_.connected() && client_.available() > 0) {
            const int raw = client_.read();
            if (raw < 0) {
                break;
            }

            const char ch = static_cast<char>(raw);
            if (ch == '\r') {
                continue;
            }

            if (ch == '\n') {
                commandBuffer_[commandLength_] = '\0';
                commandLength_ = 0;

                if (strcmp(commandBuffer_, "START_SCAN") == 0) {
                    Serial.println("Received command START_SCAN");
                    requestedScanDegrees_ = Config::kDefaultScanDegrees;
                    return RobotCommand::StartScan;
                }

                if (strncmp(commandBuffer_, "START_SCAN,", 11) == 0) {
                    const float parsedDegrees = atof(commandBuffer_ + 11);
                    if (parsedDegrees > 0.0f &&
                        parsedDegrees <= Config::kMaxScanDegrees) {
                        requestedScanDegrees_ = parsedDegrees;
                        Serial.print("Received command START_SCAN with degrees=");
                        Serial.println(requestedScanDegrees_, 2);
                        return RobotCommand::StartScan;
                    }

                    Serial.print("Ignoring invalid START_SCAN degrees: ");
                    Serial.println(commandBuffer_);
                    continue;
                }

                if (strcmp(commandBuffer_, "STOP_SCAN") == 0) {
                    Serial.println("Received command STOP_SCAN");
                    return RobotCommand::StopScan;
                }

                if (strcmp(commandBuffer_, "HARD_STOP") == 0) {
                    Serial.println("Received command HARD_STOP");
                    return RobotCommand::HardStop;
                }

                if (strcmp(commandBuffer_, "RELEASE_MOTORS") == 0) {
                    Serial.println("Received command RELEASE_MOTORS");
                    return RobotCommand::ReleaseMotors;
                }

                if (strcmp(commandBuffer_, "ZERO_TURRET") == 0) {
                    Serial.println("Received command ZERO_TURRET");
                    return RobotCommand::ZeroTurret;
                }

                if (strncmp(commandBuffer_, "MOVE_TO,", 8) == 0) {
                    const char* rest = commandBuffer_ + 8;
                    const char* comma = strchr(rest, ',');
                    if (comma != nullptr) {
                        const float parsedYaw = atof(rest);
                        const float parsedPitch = atof(comma + 1);
                        requestedMoveYawDeg_ = parsedYaw;
                        requestedMovePitchDeg_ = parsedPitch;
                        Serial.print("Received command MOVE_TO yaw=");
                        Serial.print(parsedYaw, 2);
                        Serial.print(" pitch=");
                        Serial.println(parsedPitch, 2);
                        return RobotCommand::MoveTo;
                    }
                    Serial.print("Ignoring invalid MOVE_TO command: ");
                    Serial.println(commandBuffer_);
                    continue;
                }

                if (commandBuffer_[0] != '\0') {
                    Serial.print("Ignoring unknown command: ");
                    Serial.println(commandBuffer_);
                }
                continue;
            }

            if (commandLength_ < Config::kCommandBufferSize - 1) {
                commandBuffer_[commandLength_++] = ch;
            } else {
                commandLength_ = 0;
                commandBuffer_[0] = '\0';
                Serial.println("Command buffer overflow. Dropping input line.");
            }
        }

        return RobotCommand::None;
    }

    float requestedScanDegrees() const { return requestedScanDegrees_; }
    float requestedMoveYawDeg() const { return requestedMoveYawDeg_; }
    float requestedMovePitchDeg() const { return requestedMovePitchDeg_; }

    void sendHeartbeatIfDue(const char* state, float yawDeg, float pitchDeg) {
        if (!ensureServerConnection()) {
            return;
        }

        const unsigned long nowMs = millis();
        if (nowMs - lastHeartbeatSentMs_ < Config::kHeartbeatIntervalMs) {
            return;
        }
        lastHeartbeatSentMs_ = nowMs;

        client_.print("HEARTBEAT,");
        client_.print(nowMs);
        client_.print(",");
        client_.print(state);
        client_.print(",");
        client_.print(yawDeg, 2);
        client_.print(",");
        client_.println(pitchDeg, 2);
    }

    void sendState(const char* state) {
        if (!client_.connected()) {
            return;
        }

        client_.print("STATE,");
        client_.println(state);
    }

    void sendSensorTimeout(const char* reason) {
        if (!client_.connected()) {
            return;
        }

        client_.print("SENSOR_TIMEOUT,");
        client_.println(reason);
    }

    void sendSensorStatus(const char* status) {
        if (!client_.connected()) {
            return;
        }

        client_.print("SENSOR_STATUS,");
        client_.println(status);
    }

    void sendPose(float yawDeg, float pitchDeg) {
        if (!client_.connected()) {
            return;
        }

        client_.print("POSE,");
        client_.print(yawDeg, 2);
        client_.print(",");
        client_.println(pitchDeg, 2);
    }

    void sendAbort(const char* reason, uint32_t frameId) {
        if (!client_.connected()) {
            return;
        }

        client_.print("ABORT,");
        client_.print(frameId);
        client_.print(",");
        client_.println(reason);
    }

    bool beginFrame(uint32_t frameId) {
        if (!client_.connected() && !ensureServerConnection()) {
            Serial.println("WARN: No server connection for frame start.");
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

    void discardPendingInput() {
        while (client_.connected() && client_.available() > 0) {
            client_.read();
        }
        commandLength_ = 0;
        commandBuffer_[0] = '\0';
    }

private:
    void connectWifiBlocking() {
        Serial.println("Connecting to WiFi... Looking for available networks...");
        const int networkCount = WiFi.scanNetworks();
        Serial.println("Available WiFi networks:");
        for (int i = 0; i < networkCount; ++i) {
            Serial.println(WiFi.SSID(i));
        }

        // if the SSID is not in the scan results, exit and error message 
        if (networkCount == 0) {
            Serial.println("ERROR: No WiFi networks found. Check WiFi credentials and try again.");
            while (true) {
                delay(1000);
            }
        }

        // if SSID not in
        bool ssidFound = false;
        for (int i = 0; i < networkCount; ++i) {
            if (strcmp(WiFi.SSID(i), ssid_) == 0) {
                ssidFound = true;
                break;
            }
        }
        if (!ssidFound) {
            Serial.print("ERROR: WiFi SSID '");
            Serial.print(ssid_);
            Serial.println("' not found in scan results. Check WiFi credentials and try again.");
            while (true) {
                delay(1000);
            }
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
            if (status == 6) {  // WL_CONNECT_FAILED
                Serial.println("Restart WiFi router and check credentials.");
            } else if (status == WL_NO_SSID_AVAIL) {
                Serial.println("Check WiFi SSID.");
            }
            delay(Config::kWifiRetryDelayMs);
        }

        Serial.print("WiFi connected. Board IP: ");
        Serial.println(WiFi.localIP());
    }

    void sendHello() {
        client_.println("HELLO,NANO_RP2040_CONNECT,POINT_CLOUD_V3");
        sendState("IDLE");
        sendSensorStatus("READY");
        sendPose(0.0f, Config::kPitchStartingAngleDeg);
        lastHeartbeatSentMs_ = millis();
    }

    bool connectToServer() {
        IPAddress resolvedIp;
        if (WiFi.hostByName(serverHost_, resolvedIp) == 1) {
            Serial.print("Resolved server host ");
            Serial.print(serverHost_);
            Serial.print(" to ");
            Serial.println(resolvedIp);

            if (client_.connect(resolvedIp, serverPort_)) {
                sendHello();
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

        sendHello();
        Serial.println("Server connected.");
        return true;
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
        commandLength_ = 0;
        commandBuffer_[0] = '\0';
        return connectToServer();
    }

    const char* ssid_;
    const char* password_;
    const char* serverHost_;
    IPAddress serverFallbackIp_;
    uint16_t serverPort_;
    unsigned long lastServerConnectAttemptMs_;
    unsigned long lastHeartbeatSentMs_;
    float requestedScanDegrees_;
    float requestedMoveYawDeg_;
    float requestedMovePitchDeg_;
    WiFiClient client_;
    char commandBuffer_[Config::kCommandBufferSize];
    size_t commandLength_;
};

class PointCloudScanner {
public:
    PointCloudScanner(StepperAxis& yawAxis, StepperAxis& pitchAxis,
                      DistanceSensor& distanceSensor, RobotLink& robotLink)
        : yawAxis_(yawAxis),
          pitchAxis_(pitchAxis),
          distanceSensor_(distanceSensor),
          robotLink_(robotLink),
          nextFrameId_(1) {}

    void begin() {
        yawAxis_.setSpeedRpm(Config::kYawMotorSpeedRpm);
        pitchAxis_.setSpeedRpm(Config::kPitchMotorSpeedRpm);
        yawAxis_.toStartingPosition();
        pitchAxis_.toStartingPosition();
        robotLink_.sendPose(yawAxis_.currentAngleDeg(), pitchAxis_.currentAngleDeg());
        robotLink_.sendSensorStatus("READY");
    }

    ScanOutcome runFrame(float yawSweepEndDeg) {
        const uint32_t frameId = nextFrameId_++;
        uint16_t pointIndex = 0;
        bool softStopRequested = false;
        bool hardStopRequested = false;
        bool releaseRequested = false;
        bool sensorRecoveryFailed = false;
        const char* abortReason = nullptr;

        Serial.print("Starting frame ");
        Serial.println(frameId);

        robotLink_.sendState("SCANNING");
        const bool networkEnabled = robotLink_.beginFrame(frameId);

        auto applyCommand = [&](RobotCommand command) {
            switch (command) {
                case RobotCommand::StopScan:
                    if (!softStopRequested) {
                        softStopRequested = true;
                        robotLink_.sendState("STOPPING");
                    }
                    break;
                case RobotCommand::HardStop:
                    if (!hardStopRequested) {
                        hardStopRequested = true;
                        robotLink_.sendState("HARD_STOPPING");
                    }
                    break;
                case RobotCommand::ReleaseMotors:
                    if (!releaseRequested) {
                        releaseRequested = true;
                        hardStopRequested = true;
                        robotLink_.sendState("RELEASING");
                    }
                    break;
                default:
                    break;
            }
        };

        for (float yawTarget = Config::kYawSweepStartDeg;
             yawTarget <= yawSweepEndDeg + 0.001f;
             yawTarget += Config::kYawSweepStepDeg) {
            applyCommand(robotLink_.pollCommand());
            if (hardStopRequested) {
                break;
            }

            yawAxis_.moveTo(yawTarget);

            for (float pitchTarget = Config::kPitchSweepStartDeg;
                 pitchTarget <= Config::kPitchSweepEndDeg + 0.001f;
                 pitchTarget += Config::kPitchAnglePerStepDeg) {
                applyCommand(robotLink_.pollCommand());
                if (hardStopRequested) {
                    break;
                }

                pitchAxis_.moveTo(pitchTarget);
                delay(Config::kSampleSettleDelayMs);
                robotLink_.sendPose(yawAxis_.currentAngleDeg(), pitchAxis_.currentAngleDeg());

                uint16_t distanceMm = 0;
                const char* errorText = nullptr;
                if (!distanceSensor_.readDistanceMm(distanceMm, errorText)) {
                    if (strcmp(errorText, "SENSOR_TIMEOUT") == 0) {
                        robotLink_.sendSensorStatus("RECOVERING_SENSOR");
                        robotLink_.sendSensorTimeout(errorText);

                        const char* recoveryError = nullptr;
                        if (distanceSensor_.reconnect(recoveryError)) {
                            robotLink_.sendSensorStatus("READY");
                            Serial.println("Sensor recovered after timeout.");
                        } else {
                            robotLink_.sendSensorStatus("SENSOR_RECOVERY_FAILED");
                            robotLink_.sendSensorTimeout(recoveryError != nullptr
                                                             ? recoveryError
                                                             : "SENSOR_RECOVERY_FAILED");
                            Serial.print("Sensor recovery failed: ");
                            Serial.println(recoveryError != nullptr ? recoveryError
                                                                    : "unknown");
                            sensorRecoveryFailed = true;
                            hardStopRequested = true;
                            abortReason = "SENSOR_RECOVERY_FAILED";
                        }
                    } else {
                        robotLink_.sendSensorStatus(errorText);
                    }

                    Serial.print("Skipping point at yaw=");
                    Serial.print(yawAxis_.currentAngleDeg(), 2);
                    Serial.print(" pitch=");
                    Serial.print(pitchAxis_.currentAngleDeg(), 2);
                    Serial.print(" reason=");
                    Serial.println(errorText);
                    if (sensorRecoveryFailed) {
                        break;
                    }
                    continue;
                }

                const float rawMm = static_cast<float>(distanceMm);
                const float axialMm = axisDistanceFromSensorReading(rawMm);
                const float truePitchDeg = pitchFromSensorReading(
                    axialMm,
                    pitchAxis_.currentAngleDeg()
                );
                const Point3D hitPointMm = scanPointFromSensorReading(
                    axialMm,
                    yawAxis_.currentAngleDeg(),
                    truePitchDeg
                );

                ScanPoint point{};
                point.frameId = frameId;
                point.pointIndex = pointIndex++;
                point.yawDeg = yawAxis_.currentAngleDeg();
                point.pitchDeg = truePitchDeg;
                point.distanceMm = static_cast<uint16_t>(lroundf(axialMm));
                point.positionMm = hitPointMm;

                robotLink_.sendSensorStatus("RANGE_VALID");
                robotLink_.sendPoint(point, networkEnabled);

                applyCommand(robotLink_.pollCommand());
                if (hardStopRequested) {
                    break;
                }
            }

            if (hardStopRequested || softStopRequested) {
                break;
            }
        }

        robotLink_.endFrame(frameId, pointIndex, networkEnabled);
        if (hardStopRequested) {
            robotLink_.sendAbort(sensorRecoveryFailed
                                     ? abortReason
                                     : (releaseRequested ? "RELEASE_MOTORS" : "HARD_STOP"),
                                 frameId);
            robotLink_.sendState("IDLE");
            robotLink_.sendPose(yawAxis_.currentAngleDeg(), pitchAxis_.currentAngleDeg());
            robotLink_.discardPendingInput();
            if (sensorRecoveryFailed) {
                return ScanOutcome::SensorRecoveryFailed;
            }
            return releaseRequested ? ScanOutcome::HardStoppedReleased
                                    : ScanOutcome::HardStopped;
        }

        if (softStopRequested) {
            robotLink_.sendAbort("STOP_SCAN", frameId);
            robotLink_.sendState("IDLE");
            robotLink_.sendPose(yawAxis_.currentAngleDeg(), pitchAxis_.currentAngleDeg());
            robotLink_.discardPendingInput();
            return ScanOutcome::Stopped;
        }

        robotLink_.sendState("IDLE");
        robotLink_.sendPose(yawAxis_.currentAngleDeg(), pitchAxis_.currentAngleDeg());
        robotLink_.discardPendingInput();
        return ScanOutcome::Completed;
    }

private:
    StepperAxis& yawAxis_;
    StepperAxis& pitchAxis_;
    DistanceSensor& distanceSensor_;
    RobotLink& robotLink_;
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
RobotLink kRobotLink(Config::kWifiSsid, Config::kWifiPassword, Config::kServerHost,
                     Config::kServerFallbackIp, Config::kServerPort);
PointCloudScanner kScanner(kYawAxis, kPitchAxis, kDistanceSensor, kRobotLink);

void resetToStartingPosition() {
    kPitchAxis.toStartingPosition();
    kYawAxis.toStartingPosition();
}

void releaseAllMotors() {
    kPitchAxis.release();
    kYawAxis.release();
}

void zeroAllAxes() {
    kPitchAxis.zero();
    kYawAxis.zero();
}

void setup() {
    Serial.begin(Config::kSerialBaudRate);
    Serial.println("Point cloud scanner starting up... It might take a few seconds to initialize.");
    delay(2000);

    kI2cBus.begin();
    kI2cBus.setClock(400000);

    if (!kDistanceSensor.begin()) {
        while (true) {
            delay(10);
        }
    }

    if (!kRobotLink.begin()) {
        while (true) {
            delay(10);
        }
    }

    kScanner.begin();
    Serial.println("Point cloud scanner ready. Waiting for START_SCAN.");
}

void loop() {
    const RobotCommand command = kRobotLink.pollCommand();
    if (command == RobotCommand::StartScan) {
        const ScanOutcome outcome = kScanner.runFrame(kRobotLink.requestedScanDegrees());
        if (outcome == ScanOutcome::Completed || outcome == ScanOutcome::Stopped ||
            outcome == ScanOutcome::SensorRecoveryFailed) {
            resetToStartingPosition();
            kRobotLink.sendPose(kYawAxis.currentAngleDeg(), kPitchAxis.currentAngleDeg());
        }

        if (outcome == ScanOutcome::Completed) {
            Serial.println("Scan complete. Waiting for next command.");
        } else if (outcome == ScanOutcome::Stopped) {
            Serial.println("Scan stopped. Waiting for next command.");
        } else if (outcome == ScanOutcome::SensorRecoveryFailed) {
            kRobotLink.sendState("IDLE");
            kRobotLink.sendPose(kYawAxis.currentAngleDeg(), kPitchAxis.currentAngleDeg());
            Serial.println("Sensor recovery failed. Returned to home position.");
        } else if (outcome == ScanOutcome::HardStoppedReleased) {
            releaseAllMotors();
            kRobotLink.sendPose(kYawAxis.currentAngleDeg(), kPitchAxis.currentAngleDeg());
            Serial.println("Release command received. Motors released.");
        } else {
            kRobotLink.sendPose(kYawAxis.currentAngleDeg(), kPitchAxis.currentAngleDeg());
            Serial.println("Hard stop received. Holding current position.");
        }
    } else if (command == RobotCommand::ReleaseMotors) {
        releaseAllMotors();
        kRobotLink.sendState("MOTORS_RELEASED");
        kRobotLink.sendPose(kYawAxis.currentAngleDeg(), kPitchAxis.currentAngleDeg());
        Serial.println("Motors released while idle.");
    } else if (command == RobotCommand::ZeroTurret) {
        releaseAllMotors();
        resetToStartingPosition();
        kRobotLink.sendState("ZEROED");
        kRobotLink.sendPose(kYawAxis.currentAngleDeg(), kPitchAxis.currentAngleDeg());
        Serial.println("Turret released and zeroed while idle.");
    } else if (command == RobotCommand::MoveTo) {
        kYawAxis.moveTo(kRobotLink.requestedMoveYawDeg());
        kPitchAxis.moveTo(kRobotLink.requestedMovePitchDeg());
        kRobotLink.sendState("IDLE");
        kRobotLink.sendPose(kYawAxis.currentAngleDeg(), kPitchAxis.currentAngleDeg());
        Serial.print("Moved to yaw=");
        Serial.print(kYawAxis.currentAngleDeg(), 2);
        Serial.print(" pitch=");
        Serial.println(kPitchAxis.currentAngleDeg(), 2);
    } else if (command == RobotCommand::StopScan || command == RobotCommand::HardStop) {
        Serial.println("Robot is idle. Stop command ignored.");
    }

    kRobotLink.sendHeartbeatIfDue("IDLE", kYawAxis.currentAngleDeg(),
                                  kPitchAxis.currentAngleDeg());

    delay(Config::kIdlePollDelayMs);
}

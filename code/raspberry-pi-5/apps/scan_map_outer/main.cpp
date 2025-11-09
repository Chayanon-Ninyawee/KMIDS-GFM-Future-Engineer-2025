#include "camera_module.h"
#include "combined_processor.h"
#include "lidar_module.h"
#include "lidar_processor.h"
#include "lidar_struct.h"
#include "pico2_module.h"
#include "pico2_struct.h"
#include "pid_controller.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>
#include <wiringPi.h>

// --- Global Signal Handler ---
volatile std::sig_atomic_t stop_flag = 0;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stop_flag = 1;
}

// --- Constants ---
// Pins
const int BUTTON_PIN = 16;

// Camera
const uint32_t CAM_WIDTH = 1296;
const uint32_t CAM_HEIGHT = 972;
const float CAM_HFOV = 104.0f;

const auto cameraOptionCallback = [](lccv::PiCamera &cam) {
    libcamera::ControlList &camControls = cam.getControlList();

    cam.options->video_width = CAM_WIDTH;
    cam.options->video_height = CAM_HEIGHT;
    cam.options->framerate = 30.0f;

    camControls.set(controls::AnalogueGainMode, controls::AnalogueGainModeEnum::AnalogueGainModeManual);
    camControls.set(controls::ExposureTimeMode, controls::ExposureTimeModeEnum::ExposureTimeModeManual);
    camControls.set(controls::AwbEnable, false);

    cam.options->awb_gain_r = 0.90;
    cam.options->awb_gain_b = 1.5;

    cam.options->brightness = 0.1;
    cam.options->sharpness = 1;
    cam.options->saturation = 1.5;
    cam.options->contrast = 1;
    cam.options->gain = 5;
    cam.options->shutter = 30000;
};

// Robot Control Parameters

const float TARGET_OUTER_WALL_DISTANCE = 0.23;
const float TARGET_OUTER_WALL_DISTANCE_STARTING_SECTION = 0.40;

const float PRE_TURN_FRONT_WALL_DISTANCE = 1.20f;
const auto PRE_TURN_COOLDOWN = std::chrono::milliseconds(1500);

const float TURNING_FRONT_WALL_DISTANCE = 0.50f;
const float TURNING_FRONT_WALL_DISTANCE_STARTING_SECTION = 0.70f;

const float STOP_FRONT_WALL_DISTANCE = 1.80f;
const auto STOP_DELAY = std::chrono::milliseconds(100);

const float HEADING_TOLERANCE_DEGREES = 20.0f;
const float FORWARD_MOTOR_SPEED = 3.0f;
const int TOTAL_TURNS_TO_FINISH = 8;

// PID Gains
const double HEADING_PID_P = 3.0;
const double HEADING_PID_I = 0.0;
const double HEADING_PID_D = 0.0;
const double WALL_PID_P = 180.0;
const double WALL_PID_I = 0.0;
const double WALL_PID_D = 0.0;

// --- Helper Functions ---
/**
 * @brief Normalizes an angle to the range [-180, 180] degrees.
 * @param angle The angle in degrees.
 * @return The normalized angle.
 */
float normalizeAngle(float angle) {
    angle = std::fmod(angle + 180.0f, 360.0f);
    if (angle < 0) {
        angle += 360.0f;
    }
    return angle - 180.0f;
}

// --- Main Robot Class ---

class Robot
{
public:
    enum class Mode
    {
        NORMAL,
        PRE_TURN,
        TURNING,
        PRE_STOP,
        STOP
    };

    Robot(LidarModule &lidar, Pico2Module &pico2, CameraModule &camera, Logger &scanMapLogger)
        : lidar_(lidar)
        , pico2_(pico2)
        , camera_(camera)
        , scanMapLogger_(scanMapLogger)
        , headingPid_(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D, -100.0, 100.0)
        , wallPid_(WALL_PID_P, WALL_PID_I, WALL_PID_D, -90.0, 90.0) {
        // Activate PIDs. The wall PID will be toggled by the state machine.
        headingPid_.setActive(true);
        wallPid_.setActive(true);
    }

    /**
     * @brief The main update loop for the robot's logic.
     * @param dt The time delta since the last update in seconds.
     */
    void update(float dt) {
        auto robotDataOpt = updateRobotData(dt);
        if (!robotDataOpt) {
            // Not enough data yet, do nothing.
            pico2_.setMovementInfo(0.0f, 0.0f);
            return;
        }

        RobotData robotData = *robotDataOpt;

        bool instantUpdate;
        do {
            instantUpdate = false;
            switch (mode_) {
            case Mode::NORMAL:
                std::cout << "[Mode::NORMAL]\n";
                instantUpdate = updateNormalState(robotData);
                break;
            case Mode::PRE_TURN:
                std::cout << "[Mode::PRE_TURN]\n";
                instantUpdate = updatePreTurnState(robotData);
                break;
            case Mode::TURNING:
                std::cout << "[Mode::TURNING]\n";
                instantUpdate = updateTurningState(robotData);
                break;
            case Mode::PRE_STOP:
                std::cout << "[Mode::PRE_STOP]\n";
                instantUpdate = updatePreStopState(robotData);
                break;
            case Mode::STOP:
                std::cout << "[Mode::STOP]\n";
                motorSpeed_ = 0.0f;
                steeringPercent_ = 0.0f;
                stop_flag = 1;
                break;
            }
        } while (instantUpdate && !stop_flag);

        if (mode_ != Mode::STOP) {
            calculateSteering(dt, robotData);
        }

        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
    }

private:
    // Store references to the hardware modules
    LidarModule &lidar_;
    Pico2Module &pico2_;
    CameraModule &camera_;
    Logger &scanMapLogger_;

    // PID controllers
    PIDController headingPid_;
    PIDController wallPid_;

    // Robot state
    Mode mode_ = Mode::NORMAL;
    int turnCount_ = 0;
    Direction headingDirection_ = Direction::NORTH;
    std::optional<float> initialHeading_;
    std::optional<RotationDirection> turnDirection_;

    // State-specific timing
    std::optional<std::chrono::steady_clock::time_point> lastPreTurnTimestamp_;
    std::optional<std::chrono::steady_clock::time_point> preStopTimestamp_;

    // Outputs
    float motorSpeed_ = 0.0f;
    float steeringPercent_ = 0.0f;

    struct RobotData {
        float heading;

        std::optional<lidar_processor::LineSegment> frontWall;
        std::optional<lidar_processor::LineSegment> backWall;
        std::optional<lidar_processor::LineSegment> outerWall;
        std::optional<lidar_processor::LineSegment> innerWall;
    };

    /**
     * @brief Gathers sensor data, processes it, and updates internal RobotData.
     * @param dt Delta time.
     * @return An optional RobotData struct. Returns nullopt if data is incomplete.
     */
    std::optional<RobotData> updateRobotData(float dt) {
        std::vector<TimedLidarData> timedLidarDatas;
        lidar_.getAllTimedLidarData(timedLidarDatas);
        if (timedLidarDatas.size() < lidar_.bufferSize()) return std::nullopt;

        std::vector<TimedPico2Data> timedPico2Datas;
        pico2_.getAllTimedData(timedPico2Datas);
        if (timedPico2Datas.size() < pico2_.bufferSize()) return std::nullopt;

        std::vector<TimedFrame> timedFrames;
        camera_.getAllTimedFrame(timedFrames);
        if (timedFrames.size() < camera_.bufferSize()) return std::nullopt;

        auto &timedLidarData = timedLidarDatas.back();
        auto &timedPico2Data = timedPico2Datas.back();
        auto &timedFrame = timedFrames.back();

        // Log main loop timestamp
        auto now = std::chrono::steady_clock::now();
        uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        struct {
            uint64_t lidarTimestamp_ns;
            uint64_t pico2Timestamp_ns;
            uint64_t cameraTimestamp_ns;
        } scanMapData{
            static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timedLidarData.timestamp.time_since_epoch()).count()
            ),
            static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timedPico2Data.timestamp.time_since_epoch()).count()
            ),
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(timedFrame.timestamp.time_since_epoch()).count())
        };
        scanMapLogger_.writeData(timestamp_ns, &scanMapData, sizeof(scanMapData));

        if (!initialHeading_) {
            initialHeading_ = timedPico2Data.euler.h;
            return std::nullopt;
        }

        RobotData data;
        data.heading = timedPico2Datas.back().euler.h - *initialHeading_;
        data.heading = std::fmod(data.heading, 360.0f);
        if (data.heading < 0.0f) data.heading += 360.0f;

        auto filteredLidarData = lidar_processor::filterLidarData(timedLidarDatas.back());
        auto deltaPose = combined_processor::aproximateRobotPose(filteredLidarData, timedPico2Datas);
        auto lineSegments = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        auto relativeWalls = lidar_processor::getRelativeWalls(lineSegments, headingDirection_, data.heading, 0.30f, 25.0f, 0.22f);
        auto resolvedWalls = lidar_processor::resolveWalls(relativeWalls);

        if (!turnDirection_) {
            turnDirection_ = lidar_processor::getTurnDirection(relativeWalls);
        }

        data.frontWall = resolvedWalls.frontWall;
        data.backWall = resolvedWalls.backWall;

        if (turnDirection_) {
            if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                data.outerWall = resolvedWalls.leftWall;
                data.innerWall = resolvedWalls.rightWall;
            } else {
                data.outerWall = resolvedWalls.rightWall;
                data.innerWall = resolvedWalls.leftWall;
            }
        }
        return data;
    }

    // --- State Handlers ---

    bool updateNormalState(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;

        if (turnCount_ >= TOTAL_TURNS_TO_FINISH) {
            mode_ = Mode::PRE_STOP;
            return true;  // Instant update
        }

        auto now = std::chrono::steady_clock::now();
        bool cooldownOver = !lastPreTurnTimestamp_ || (now - *lastPreTurnTimestamp_ >= PRE_TURN_COOLDOWN);

        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= PRE_TURN_FRONT_WALL_DISTANCE && cooldownOver) {
            mode_ = Mode::PRE_TURN;
            lastPreTurnTimestamp_ = now;
            return true;  // Instant update
        }
        return false;
    }

    bool updatePreTurnState(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        // Wall PID is active during this state.

        float turningFrontWallDistance;
        if (turnCount_ % 4 == 3) {
            turningFrontWallDistance = TURNING_FRONT_WALL_DISTANCE_STARTING_SECTION;
        } else {
            turningFrontWallDistance = TURNING_FRONT_WALL_DISTANCE;
        }

        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= turningFrontWallDistance) {
            float turnAngle = (turnDirection_.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) ? 90.0f : -90.0f;
            float nextHeading = std::fmod(headingDirection_.toHeading() + turnAngle + 360.0f, 360.0f);
            headingDirection_ = Direction::fromHeading(nextHeading);

            mode_ = Mode::TURNING;
            return true;  // Instant update
        }
        return false;
    }

    bool updateTurningState(const RobotData &data) {
        wallPid_.setActive(false);  // Disable wall PID during the turn.
        motorSpeed_ = FORWARD_MOTOR_SPEED;

        float diff = normalizeAngle(data.heading - headingDirection_.toHeading());
        if (std::abs(diff) <= HEADING_TOLERANCE_DEGREES) {
            turnCount_++;
            wallPid_.setActive(true);  // Re-activate wall following after the turn is complete.
            mode_ = Mode::NORMAL;
            return true;  // Instant update
        }
        return false;
    }

    bool updatePreStopState(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;

        if (!preStopTimestamp_) {
            preStopTimestamp_ = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - *preStopTimestamp_;
        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= STOP_FRONT_WALL_DISTANCE && elapsed >= STOP_DELAY) {
            preStopTimestamp_.reset();  // Clear timestamp
            mode_ = Mode::STOP;
            return true;  // Instant update
        }
        return false;
    }

    // --- PID and Steering Calculation ---

    void calculateSteering(float dt, const RobotData &data) {
        float headingError = normalizeAngle(headingDirection_.toHeading() - data.heading);

        float wallError = 0.0f;
        if (data.outerWall) {
            float targetOuterWallDistance;
            if (turnCount_ % 4 == 0) {
                targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE_STARTING_SECTION;
            } else {
                targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE;
            }

            wallError = data.outerWall->perpendicularDistance(0.0f, 0.0f) - targetOuterWallDistance;
        }
        // This will return 0.0 if the wall PID is inactive, effectively skipping it
        float headingErrorOffset = wallPid_.update(wallError, dt);

        if (motorSpeed_ < 0) headingErrorOffset = -headingErrorOffset;

        if (turnDirection_.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) {
            headingError -= headingErrorOffset;
        } else {
            headingError += headingErrorOffset;
        }

        if (motorSpeed_ < 0) headingError = -headingError;

        steeringPercent_ = headingPid_.update(headingError, dt);
    }
};

int main() {
    std::signal(SIGINT, signalHandler);

    // --- Setup Logging ---
    const char *home = std::getenv("HOME");
    if (!home) {
        std::cerr << "HOME environment variable not set" << std::endl;
        return -1;
    }
    std::string logFolder = std::string(home) + "/gfm_logs/scan_map_outer";
    std::string timedstampedLogFolder = Logger::generateTimestampedFolder(logFolder);

    // Create logger instances on the stack
    Logger lidarLogger(timedstampedLogFolder + "/lidar.bin");
    Logger pico2Logger(timedstampedLogFolder + "/pico2.bin");
    Logger cameraLogger(timedstampedLogFolder + "/camera.bin");
    Logger openChallengeLogger(timedstampedLogFolder + "/scanMap.bin");

    // --- Initialize Hardware Modules ---
    LidarModule lidar(&lidarLogger);
    Pico2Module pico2(&pico2Logger);
    CameraModule camera(&cameraLogger, cameraOptionCallback);

    if (!lidar.initialize()) {
        std::cerr << "LidarModule initialization failed." << std::endl;
        return -1;
    }
    lidar.printDeviceInfo();
    if (!lidar.start()) {
        std::cerr << "LidarModule failed to start." << std::endl;
        return -1;
    }
    if (!pico2.initialize()) {
        std::cerr << "Pico2Module initialization failed." << std::endl;
        return -1;
    }
    if (!camera.start()) {
        std::cerr << "CameraModule failed to start." << std::endl;
        return -1;
    }

    // --- Initialize Robot Controller ---
    Robot robot(lidar, pico2, camera, openChallengeLogger);  // Pass modules by reference

    // --- Setup GPIO ---
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "WiringPi setup failed." << std::endl;
        return -1;
    }
    pinMode(BUTTON_PIN, INPUT);
    pullUpDnControl(BUTTON_PIN, PUD_UP);

    // --- Wait for Start Button ---
    std::cout << "Press the button to start..." << std::endl;
    while (digitalRead(BUTTON_PIN) == HIGH && !stop_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!stop_flag) {
        std::cout << "Starting in 1.5 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        lidar.startLogging();
        pico2.startLogging();
        camera.startLogging();

        // --- Main Loop ---
        const auto loopDuration = std::chrono::milliseconds(32);  // ~30 Hz
        auto lastTime = std::chrono::steady_clock::now();

        std::cout << "Robot running." << std::endl;
        while (!stop_flag) {
            auto loopStart = std::chrono::steady_clock::now();

            float dt = std::chrono::duration<float>(loopStart - lastTime).count();
            lastTime = loopStart;

            robot.update(dt);

            // Maintain loop rate
            auto loopEnd = std::chrono::steady_clock::now();
            auto elapsed = loopEnd - loopStart;
            if (elapsed < loopDuration) {
                std::this_thread::sleep_for(loopDuration - elapsed);
            }
        }
    } else {
        // If stopped before starting, clean up the created log folder
        std::filesystem::remove_all(timedstampedLogFolder);
        std::cout << "Start aborted. Log folder removed." << std::endl;
    }

    // --- Shutdown ---
    std::cout << "Shutting down..." << std::endl;
    pico2.setMovementInfo(0.0f, 0.0f);  // Ensure motors are stopped
    lidar.stop();
    lidar.shutdown();
    pico2.shutdown();
    std::cout << "Shutdown complete." << std::endl;

    return 0;
}

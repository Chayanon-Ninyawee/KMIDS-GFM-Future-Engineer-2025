#include "camera_module.h"
#include "combined_processor.h"
#include "lidar_module.h"
#include "lidar_processor.h"
#include "pico2_module.h"
#include "pid_controller.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <optional>
#include <thread>

volatile std::sig_atomic_t stop_flag = 0;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stop_flag = 1;
}

const uint32_t camWidth = 1296;
const uint32_t camHeight = 972;

const float TARGET_OUTER_WALL_DISTANCE = 0.23;
const float TARGET_OUTER_WALL_DISTANCE_STARTING_SECTION = 0.40;

const float PRE_TURN_FRONT_WALL_DISTANCE = 1.20f;
const auto PRE_TURN_COOLDOWN = std::chrono::milliseconds(4000);

const float TURNING_FRONT_WALL_DISTANCE = 0.50f;
const float TURNING_FRONT_WALL_DISTANCE_STARTING_SECTION = 0.70f;

const float STOP_FRONT_WALL_DISTANCE = 1.80f;
const auto STOP_DELAY = std::chrono::milliseconds(100);

enum Mode
{
    NORMAL,
    PRE_TURN,
    TURNING,
    PRE_STOP,
    STOP
};

struct State {
    PIDController headingPid{3.0f, 0.0, 0.0f};
    PIDController wallPid{180.0f, 0.0, 0.0f};

    std::optional<float> initialHeading;
    std::optional<RotationDirection> robotTurnDirection = RotationDirection::COUNTER_CLOCKWISE;

    Mode robotMode = Mode::NORMAL;
    int numberOfTurn = 0;
    Direction headingDirection = Direction::NORTH;
};

void update(float dt, LidarModule &lidar, Pico2Module &pico2, State &state, float &outMotorSpeed, float &outSteeringPercent) {
    std::vector<TimedLidarData> timedLidarDatas;
    lidar.getAllTimedLidarData(timedLidarDatas);
    if (timedLidarDatas.empty()) return;

    std::vector<TimedPico2Data> timedPico2Datas;
    pico2.getAllTimedData(timedPico2Datas);
    if (timedPico2Datas.empty()) return;

    auto &timedLidarData = timedLidarDatas[timedLidarDatas.size() - 1];
    auto &timedPico2Data = timedPico2Datas[timedPico2Datas.size() - 1];

    auto now = std::chrono::steady_clock::now();

    auto filteredLidarData = lidar_processor::filterLidarData(timedLidarData);

    auto deltaPose = combined_processor::aproximateRobotPose(filteredLidarData, timedPico2Datas);

    // std::cout << "[DeltaPose] ΔX: " << deltaPose.deltaX << " m, ΔY: " << deltaPose.deltaY << " m, ΔH: " << deltaPose.deltaH << " deg"
    //           << std::endl;

    if (not state.initialHeading) state.initialHeading = timedPico2Data.euler.h;

    float heading = timedPico2Data.euler.h - state.initialHeading.value_or(0.0f);
    heading = std::fmod(heading, 360.0f);
    if (heading < 0.0f) heading += 360.0f;

    auto lineSegments = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
    auto relativeWalls = lidar_processor::getRelativeWalls(lineSegments, state.headingDirection, heading, 0.30f, 25.0f, 0.22f);
    auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);

    // Set robotTurnDirection
    if (!state.robotTurnDirection) {
        auto newRobotTurnDirection = lidar_processor::getTurnDirection(relativeWalls);
        if (newRobotTurnDirection) state.robotTurnDirection = newRobotTurnDirection;
    }

    auto frontWall = resolveWalls.frontWall;
    auto backWall = resolveWalls.backWall;

    std::optional<lidar_processor::LineSegment> outerWall;
    std::optional<lidar_processor::LineSegment> innerWall;

    if (state.robotTurnDirection) {
        if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
            outerWall = resolveWalls.leftWall;
            innerWall = resolveWalls.rightWall;
        } else {
            outerWall = resolveWalls.rightWall;
            innerWall = resolveWalls.leftWall;
        }
    }

    bool pidWallErrorActive = true;

    float targetOuterWallDistance;
    if (state.numberOfTurn % 4 == 0) {
        targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE_STARTING_SECTION;
    } else {
        targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE;
    }

instant_update:
    switch (state.robotMode) {
    default:
        std::cout << "[ScanMapOuter] Invalid Mode!" << std::endl;
        stop_flag = 1;
        return;

    case Mode::NORMAL: {
        // std::cout << "[Mode::NORMAL]\n";

        outMotorSpeed = 2.5f;

        if (state.numberOfTurn == 8) {
            state.robotMode = Mode::PRE_STOP;
            goto instant_update;
        }

        static auto lastPreTurnTrigger = std::chrono::steady_clock::now() - PRE_TURN_COOLDOWN;
        if (frontWall && frontWall->perpendicularDistance(0.0f, 0.0f) <= PRE_TURN_FRONT_WALL_DISTANCE &&
            (now - lastPreTurnTrigger) >= PRE_TURN_COOLDOWN)
        {
            state.robotMode = Mode::PRE_TURN;
            lastPreTurnTrigger = now;
            goto instant_update;
        }
        break;
    }
    case Mode::PRE_TURN: {
        // std::cout << "[Mode::PRE_TURN]\n";

        outMotorSpeed = 2.5f;

        float turningFrontWallDistance;
        if (state.numberOfTurn % 4 == 3) {
            turningFrontWallDistance = TURNING_FRONT_WALL_DISTANCE_STARTING_SECTION;
        } else {
            turningFrontWallDistance = TURNING_FRONT_WALL_DISTANCE;
        }

        if (frontWall && frontWall->perpendicularDistance(0.0f, 0.0f) <= turningFrontWallDistance) {
            Direction nextHeadingDirection;
            if (state.robotTurnDirection.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) {
                float nextHeading = state.headingDirection.toHeading() + 90.0f;
                nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
                nextHeadingDirection = Direction::fromHeading(nextHeading);
            } else {
                float nextHeading = state.headingDirection.toHeading() - 90.0f;
                nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
                nextHeadingDirection = Direction::fromHeading(nextHeading);
            }

            state.headingDirection = nextHeadingDirection;

            state.robotMode = Mode::TURNING;
            goto instant_update;
        }
        break;
    }
    case Mode::TURNING: {
        // std::cout << "[Mode::TURNING]\n";

        outMotorSpeed = 2.5f;

        pidWallErrorActive = false;

        float diff = heading - state.headingDirection.toHeading();
        diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        if (std::abs(diff) <= 20.0f) {
            state.numberOfTurn++;
            state.robotMode = Mode::NORMAL;
            goto instant_update;
        }
        break;
    }
    case Mode::PRE_STOP: {
        // std::cout << "[Mode::PRE_STOP]\n";

        static bool stopTimerActive = false;
        static auto stopStartTime = std::chrono::steady_clock::now();
        if (!stopTimerActive) {
            stopTimerActive = true;
            stopStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - stopStartTime;
        if (frontWall && frontWall->perpendicularDistance(0.0f, 0.0f) <= STOP_FRONT_WALL_DISTANCE && elapsed >= STOP_DELAY) {
            stopTimerActive = false;

            state.robotMode = Mode::STOP;
            goto instant_update;
        }
        outMotorSpeed = 2.5f;

        break;
    }
    case Mode::STOP: {
        // std::cout << "[Mode::STOP]\n";
        outMotorSpeed = 0.0f;
        outSteeringPercent = 0.0f;

        stop_flag = 1;
        return;
    }
    }

    // NOTE: Use break and it will run the pid
    // If don't want to run the pid then use return

    float wallError = 0.0f;
    if (outerWall) {
        wallError = outerWall->perpendicularDistance(0.0f, 0.0f) - targetOuterWallDistance;
    }
    float headingErrorOffset = state.wallPid.update(wallError, dt);

    float headingError = state.headingDirection.toHeading() - heading;
    headingError = std::fmod(headingError + 180.0f, 360.0f);
    if (headingError < 0) headingError += 360.0f;
    headingError -= 180.0f;

    if (pidWallErrorActive) {
        if (state.robotTurnDirection.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) {
            headingError -= headingErrorOffset;
        } else {
            headingError += headingErrorOffset;
        }
    }

    outSteeringPercent = state.headingPid.update(headingError, dt);

    // std::cout << "Heading: " << heading << "°, Heading Direction: " << state.headingDirection.toHeading()
    //           << "°, Heading Error: " << headingError << "°, Heading Error Offset: " << headingErrorOffset
    //           << "°, Motor Speed: " << outMotorSpeed << "rps, Steering Percent: " << outSteeringPercent << "%\n"
    //           << std::endl;

    // auto dir = state.robotTurnDirection.value_or(RotationDirection::CLOCKWISE);
    // std::cout << "robotTurnDirection: " << (dir == RotationDirection::CLOCKWISE ? "CLOCKWISE" : "COUNTERCLOCKWISE") << "\n";
}

int main() {
    std::signal(SIGINT, signalHandler);

    const char *home = std::getenv("HOME");
    if (!home) throw std::runtime_error("HOME environment variable not set");
    std::string logFolder = std::string(home) + "/gfm_logs/scan_map_outer";

    Logger lidarLogger(Logger::generateFilename(logFolder, "lidar"));
    Logger pico2Logger(Logger::generateFilename(logFolder, "pico2"));
    Logger cameraLogger(Logger::generateFilename(logFolder, "camera"));

    // Initialize LidarModule
    LidarModule lidar(&lidarLogger);
    if (!lidar.initialize()) return -1;
    lidar.printDeviceInfo();
    if (!lidar.start()) return -1;

    // Initialize Pico2Module
    Pico2Module pico2(&pico2Logger);
    if (!pico2.initialize()) return -1;

    // Initialize CameraModule (for logging only)
    auto cameraOptionCallback = [](lccv::PiCamera &cam) {
        libcamera::ControlList &camControls = cam.getControlList();

        cam.options->video_width = camWidth;
        cam.options->video_height = camHeight;
        cam.options->framerate = 30.0f;

        camControls.set(controls::AnalogueGainMode, controls::AnalogueGainModeEnum::AnalogueGainModeManual);
        camControls.set(controls::ExposureTimeMode, controls::ExposureTimeModeEnum::ExposureTimeModeAuto);
        camControls.set(controls::AwbEnable, false);

        cam.options->awb_gain_r = 0.83;
        cam.options->awb_gain_b = 1.5;

        cam.options->brightness = 0.1;
        cam.options->sharpness = 1;
        cam.options->saturation = 1.5;
        cam.options->contrast = 1;
        cam.options->gain = 5;
    };
    CameraModule camera(&cameraLogger, cameraOptionCallback);
    if (!camera.start()) return -1;

    State state;

    std::cout << "Waiting 2 seconds before starting control loop..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    const auto loopDuration = std::chrono::milliseconds(32);  // ~30 Hz
    auto lastTime = std::chrono::steady_clock::now();

    while (!stop_flag) {
        auto loopStart = std::chrono::steady_clock::now();

        std::chrono::duration<float> delta = loopStart - lastTime;
        float dt = delta.count();
        lastTime = loopStart;

        float motorSpeed, steeringPercent;
        update(dt, lidar, pico2, state, motorSpeed, steeringPercent);
        pico2.setMovementInfo(motorSpeed, steeringPercent);

        // Maintain ~30 Hz loop rate
        auto loopEnd = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loopEnd - loopStart);
        if (elapsed < loopDuration) {
            std::this_thread::sleep_for(loopDuration - elapsed);
        }
    }
    pico2.setMovementInfo(0.0f, 0.0f);

    // Shutdown
    lidar.stop();
    lidar.shutdown();
    pico2.shutdown();
    camera.stop();

    std::cout << "[Main] Shutdown complete." << std::endl;
    return 0;
}

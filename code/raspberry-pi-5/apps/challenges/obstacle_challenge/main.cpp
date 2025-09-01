// FIXME: UNTESTED

#include "camera_module.h"
#include "camera_processor.h"
#include "combined_processor.h"
#include "direction.h"
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
const float camHFov = 104.0f;

const float TARGET_OUTER_WALL_DISTANCE = 0.50f;
const float TARGET_OUTER_WALL_DISTANCE_PARKING = 0.30f;

const float PRE_TURN_FRONT_WALL_DISTANCE = 1.20f;
const auto PRE_TURN_COOLDOWN = std::chrono::milliseconds(1500);

const float TURNING_FRONT_WALL_DISTANCE = 0.85f;

const float STOP_FRONT_WALL_DISTANCE = 1.80f;
const auto STOP_DELAY = std::chrono::milliseconds(100);

enum Mode
{
    NORMAL,
    PRE_TURN,
    TURNING,
    FIND_PARKING,
    PARKING_1,
    PARKING_2,
    PARKING_3,
    STOP
};

struct State {
    PIDController headingPid{3.0f, 0.0, 0.0f};
    PIDController wallPid{180.0f, 0.0, 0.0f};

    std::optional<float> initialHeading;
    // FIXME: Change this back
    // std::optional<RotationDirection> robotTurnDirection;
    std::optional<RotationDirection> robotTurnDirection = RotationDirection::COUNTER_CLOCKWISE;

    // FIXME: Change this back
    // Mode robotMode = Mode::NORMAL;
    Mode robotMode = Mode::FIND_PARKING;
    int numberOfTurn = 0;
    Direction headingDirection = Direction::NORTH;
};

void update(
    float dt,
    LidarModule &lidar,
    Pico2Module &pico2,
    CameraModule &camera,
    State &state,
    float &outMotorSpeed,
    float &outSteeringPercent
) {
    std::vector<TimedLidarData> timedLidarDatas;
    lidar.getAllTimedLidarData(timedLidarDatas);
    if (timedLidarDatas.size() < 10) return;

    std::vector<TimedPico2Data> timedPico2Datas;
    pico2.getAllTimedData(timedPico2Datas);
    if (timedPico2Datas.size() < 120) return;

    std::vector<TimedFrame> timedFrames;
    camera.getAllTimedFrame(timedFrames);
    if (timedFrames.size() < 30) return;

    auto &timedLidarData = timedLidarDatas[timedLidarDatas.size() - 1];
    auto &timedPico2Data = timedPico2Datas[timedPico2Datas.size() - 1];
    auto &timedFrame = timedFrames[timedFrames.size() - 1];

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

    auto trafficLightPoints = lidar_processor::getTrafficLightPoints(filteredLidarData, resolveWalls, state.robotTurnDirection);

    auto colorMasks = camera_processor::filterColors(timedFrame);
    auto blockAngles = camera_processor::computeBlockAngles(colorMasks, camWidth, camHFov);

    auto trafficLightInfos = combined_processor::combineTrafficLightInfo(blockAngles, trafficLightPoints);

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

    float targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE;

    // TODO: Change this to match with obstacle_challenge
instant_update:
    switch (state.robotMode) {
    default:
        std::cout << "[OpenChallenge] Invalid Mode!" << std::endl;
        stop_flag = 1;
        return;

    case Mode::FIND_PARKING: {
        // FIXME: This is only for test when the parking wall is directly on the side of the robot
        auto lineSegmentsForParking = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        auto parkingWalls = lidar_processor::getParkingWalls(lineSegmentsForParking, state.headingDirection, heading, 0.30f);

        outMotorSpeed = 1.0f;
        targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE_PARKING;

        if (parkingWalls.empty()) break;

        lidar_processor::LineSegment backParkingWall;
        std::vector<lidar_processor::LineSegment> frontWalls;
        std::vector<lidar_processor::LineSegment> backWalls;

        for (auto &wall : parkingWalls) {
            if (wall.perpendicularDistance(0.0f, 0.0f) >= 1.00f) continue;

            float dir = wall.perpendicularDirection(0.0f, 0.0f);
            if (dir >= 0.0f && dir < 180.0f) {
                frontWalls.push_back(wall);
            } else {
                backWalls.push_back(wall);
            }
        }

        if (!frontWalls.empty() && !backWalls.empty()) {
            // Rule 1: if wall in front exists, pick closest back wall
            backParkingWall = *std::min_element(backWalls.begin(), backWalls.end(), [](const auto &a, const auto &b) {
                return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
            });
        } else if (!frontWalls.empty()) {
            // Rule 2: only front walls → pick closest front wall
            backParkingWall = *std::min_element(frontWalls.begin(), frontWalls.end(), [](const auto &a, const auto &b) {
                return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
            });
        } else if (!backWalls.empty()) {
            // Rule 3: only back walls → pick furthest back wall
            backParkingWall = *std::max_element(backWalls.begin(), backWalls.end(), [](const auto &a, const auto &b) {
                return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
            });
        } else {
            break;
        }

        float backParkingWallDir = backParkingWall.perpendicularDirection(0.0f, 0.0f);
        float backParkingWallDist = -backParkingWall.y2;

        std::cout << "[BackParkingWall] dir=" << backParkingWallDir << "°, dist=" << backParkingWallDist << " m" << std::endl;

        // FIXME: THE 270 AND 330 IS FOR TESTING ONLY
        bool isBackParkingWallBehind = backParkingWallDir >= 270.0f && backParkingWallDir < 330.0f;

        if (isBackParkingWallBehind && backParkingWallDist >= 0.475f) {
            state.robotMode = Mode::PARKING_1;
            goto instant_update;
        }

        break;
    }

    case Mode::PARKING_1: {
        outMotorSpeed = 0.0f;
        outSteeringPercent = 0.0f;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (elapsed < std::chrono::milliseconds(300)) return;
        outSteeringPercent = 100.0f;
        if (elapsed < std::chrono::milliseconds(600)) return;

        outMotorSpeed = -1.0f;

        static bool encoderStarted = false;
        static double startEncoderAngle = 0.0;

        if (!encoderStarted) {
            startEncoderAngle = timedPico2Data.encoderAngle;
            encoderStarted = true;
        }

        std::cout << "[PARKING_1] encoderAngle=" << timedPico2Data.encoderAngle << " startEncoderAngle=" << startEncoderAngle
                  << " delta=" << (timedPico2Data.encoderAngle - startEncoderAngle) << std::endl;

        if (timedPico2Data.encoderAngle - startEncoderAngle <= -530) {
            waitTimerActive = false;
            encoderStarted = false;

            state.robotMode = Mode::PARKING_2;
            goto instant_update;
        }

        return;
    }

    case Mode::PARKING_2: {
        outMotorSpeed = 0.0f;
        outSteeringPercent = 0.0f;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (elapsed < std::chrono::milliseconds(300)) return;
        outSteeringPercent = -100.0f;
        if (elapsed < std::chrono::milliseconds(600)) return;

        outMotorSpeed = -1.0f;

        static bool encoderStarted = false;
        static double startEncoderAngle = 0.0;

        if (!encoderStarted) {
            startEncoderAngle = timedPico2Data.encoderAngle;
            encoderStarted = true;
        }

        std::cout << "[PARKING_2] encoderAngle=" << timedPico2Data.encoderAngle << " startEncoderAngle=" << startEncoderAngle
                  << " delta=" << (timedPico2Data.encoderAngle - startEncoderAngle) << std::endl;

        if (timedPico2Data.encoderAngle - startEncoderAngle <= -380) {
            waitTimerActive = false;
            encoderStarted = false;

            state.robotMode = Mode::PARKING_3;
            goto instant_update;
        }

        return;
    }

    case Mode::PARKING_3: {
        outMotorSpeed = 0.0f;
        outSteeringPercent = 0.0f;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (elapsed < std::chrono::milliseconds(300)) return;
        outSteeringPercent = 100.0f;
        if (elapsed < std::chrono::milliseconds(600)) return;

        outMotorSpeed = 1.0f;

        static bool encoderStarted = false;
        static double startEncoderAngle = 0.0;

        if (!encoderStarted) {
            startEncoderAngle = timedPico2Data.encoderAngle;
            encoderStarted = true;
        }

        std::cout << "[PARKING_3] encoderAngle=" << timedPico2Data.encoderAngle << " startEncoderAngle=" << startEncoderAngle
                  << " delta=" << (timedPico2Data.encoderAngle - startEncoderAngle) << std::endl;

        if (timedPico2Data.encoderAngle - startEncoderAngle >= 92) {
            waitTimerActive = false;
            encoderStarted = false;

            state.robotMode = Mode::STOP;
            goto instant_update;
        }

        return;
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

    if (state.robotTurnDirection.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) {
        headingError -= headingErrorOffset;
    } else {
        headingError += headingErrorOffset;
    }

    outSteeringPercent = state.headingPid.update(headingError, dt);

    // std::cout << "Heading: " << heading << "°, Heading Direction: " << state.headingDirection.toHeading()
    //           << "°, Heading Error: " << headingError << "°, Heading Error Offset: " << headingErrorOffset
    //           << "°, Motor Speed: " << outMotorSpeed << "rps, Steering Percent: " << outSteeringPercent << "%\n"
    //           << std::endl;

    auto dir = state.robotTurnDirection.value_or(RotationDirection::CLOCKWISE);
    // std::cout << "robotTurnDirection: " << (dir == RotationDirection::CLOCKWISE ? "CLOCKWISE" : "COUNTERCLOCKWISE") << "\n";
}

int main() {
    std::signal(SIGINT, signalHandler);

    const char *home = std::getenv("HOME");
    if (!home) throw std::runtime_error("HOME environment variable not set");
    std::string logFolder = std::string(home) + "/gfm_logs/obstacle_challenge";

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
    //
    // Initialize CameraModule
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

    const auto loopDuration = std::chrono::milliseconds(16);  // ~60 Hz
    auto lastTime = std::chrono::steady_clock::now();

    while (!stop_flag) {
        auto loopStart = std::chrono::steady_clock::now();

        std::chrono::duration<float> delta = loopStart - lastTime;
        float dt = delta.count();
        lastTime = loopStart;

        float motorSpeed, steeringPercent;
        update(dt, lidar, pico2, camera, state, motorSpeed, steeringPercent);
        pico2.setMovementInfo(motorSpeed, steeringPercent);

        // Maintain ~60 Hz loop rate
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

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
#include <iostream>
#include <optional>
#include <thread>
#include <wiringPi.h>

volatile std::sig_atomic_t stop_flag = 0;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stop_flag = 1;
}

const int BUTTON_PIN = 16;

const float TARGET_OUTER_WALL_DISTANCE = 0.30f;

const float PRE_TURN_FRONT_WALL_DISTANCE = 1.20f;
const auto PRE_TURN_COOLDOWN = std::chrono::milliseconds(1500);

const float TURNING_FRONT_WALL_DISTANCE = 0.65f;

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
    std::optional<RotationDirection> robotTurnDirection;

    Mode robotMode = Mode::NORMAL;
    int numberOfTurn = 0;
    Direction headingDirection = Direction::NORTH;
};

struct RobotData {
    std::vector<TimedLidarData> lidarDatas;
    TimedLidarData latestLidarData;

    std::vector<TimedPico2Data> pico2Datas;
    TimedPico2Data latestPico2Data;

    float heading;

    std::optional<lidar_processor::LineSegment> frontWall;
    std::optional<lidar_processor::LineSegment> backWall;
    std::optional<lidar_processor::LineSegment> outerWall;
    std::optional<lidar_processor::LineSegment> innerWall;
};

struct StateReturn {
    bool instantUpdate = false;
    bool skipPid = false;
    bool skipWallPid = false;
};

std::optional<RobotData> updateRobotData(float dt, LidarModule &lidar, Pico2Module &pico2, State &state) {
    RobotData robotData;

    lidar.getAllTimedLidarData(robotData.lidarDatas);
    if (robotData.lidarDatas.size() < lidar.bufferSize()) return std::nullopt;

    pico2.getAllTimedData(robotData.pico2Datas);
    if (robotData.pico2Datas.size() < pico2.bufferSize()) return std::nullopt;

    if (not state.initialHeading) state.initialHeading = robotData.pico2Datas.back().euler.h;
    robotData.heading = robotData.pico2Datas.back().euler.h - state.initialHeading.value_or(0.0f);
    robotData.heading = std::fmod(robotData.heading, 360.0f);
    if (robotData.heading < 0.0f) robotData.heading += 360.0f;

    auto filteredLidarData = lidar_processor::filterLidarData(robotData.lidarDatas.back());

    auto deltaPose = combined_processor::aproximateRobotPose(filteredLidarData, robotData.pico2Datas);

    // std::cout << "[DeltaPose] ΔX: " << deltaPose.deltaX << " m, ΔY: " << deltaPose.deltaY << " m, ΔH: " << deltaPose.deltaH << " deg"
    //           << std::endl;

    auto lineSegments = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
    auto relativeWalls = lidar_processor::getRelativeWalls(lineSegments, state.headingDirection, robotData.heading, 0.30f, 25.0f, 0.22f);
    auto resolveWalls = lidar_processor::resolveWalls(relativeWalls);

    // Set robotTurnDirection
    if (!state.robotTurnDirection) {
        auto newRobotTurnDirection = lidar_processor::getTurnDirection(relativeWalls);
        if (newRobotTurnDirection) state.robotTurnDirection = newRobotTurnDirection;
    }

    robotData.frontWall = resolveWalls.frontWall;
    robotData.backWall = resolveWalls.backWall;
    if (state.robotTurnDirection) {
        if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
            robotData.outerWall = resolveWalls.leftWall;
            robotData.innerWall = resolveWalls.rightWall;
        } else {
            robotData.outerWall = resolveWalls.rightWall;
            robotData.innerWall = resolveWalls.leftWall;
        }
    }

    return robotData;
}

StateReturn stateNormal(RobotData &robotData, State &state, float &outMotorSpeed, float &outSteeringPercent) {
    StateReturn stateReturn;

    auto now = std::chrono::steady_clock::now();

    // std::cout << "[Mode::NORMAL]\n";

    outMotorSpeed = 4.5f;

    if (state.numberOfTurn == 12) {
        state.robotMode = Mode::PRE_STOP;

        stateReturn.instantUpdate = true;
        return stateReturn;
    }

    static auto lastPreTurnTrigger = std::chrono::steady_clock::now() - PRE_TURN_COOLDOWN;
    if (robotData.frontWall && robotData.frontWall->perpendicularDistance(0.0f, 0.0f) <= PRE_TURN_FRONT_WALL_DISTANCE &&
        (now - lastPreTurnTrigger) >= PRE_TURN_COOLDOWN)
    {
        state.robotMode = Mode::PRE_TURN;
        lastPreTurnTrigger = now;

        stateReturn.instantUpdate = true;
        return stateReturn;
    }

    return stateReturn;
}

StateReturn statePreTurn(RobotData &robotData, State &state, float &outMotorSpeed, float &outSteeringPercent) {
    StateReturn stateReturn;

    // std::cout << "[Mode::PRE_TURN]\n";

    outMotorSpeed = 4.5f;

    if (robotData.frontWall && robotData.frontWall->perpendicularDistance(0.0f, 0.0f) <= TURNING_FRONT_WALL_DISTANCE) {
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

        stateReturn.instantUpdate = true;
        return stateReturn;
    }

    return stateReturn;
}

StateReturn stateTurning(RobotData &robotData, State &state, float &outMotorSpeed, float &outSteeringPercent) {
    StateReturn stateReturn;

    // std::cout << "[Mode::TURNING]\n";

    outMotorSpeed = 4.5f;

    stateReturn.skipWallPid = true;

    float diff = robotData.heading - state.headingDirection.toHeading();
    diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
    if (std::abs(diff) <= 20.0f) {
        state.numberOfTurn++;
        state.robotMode = Mode::NORMAL;

        stateReturn.instantUpdate = true;
        return stateReturn;
    }

    return stateReturn;
}

StateReturn statePreStop(RobotData &robotData, State &state, float &outMotorSpeed, float &outSteeringPercent) {
    StateReturn stateReturn;

    // std::cout << "[Mode::PRE_STOP]\n";

    outMotorSpeed = 4.5f;

    static bool stopTimerActive = false;
    static auto stopStartTime = std::chrono::steady_clock::now();
    if (!stopTimerActive) {
        stopTimerActive = true;
        stopStartTime = std::chrono::steady_clock::now();
    }

    auto elapsed = std::chrono::steady_clock::now() - stopStartTime;
    if (robotData.frontWall && robotData.frontWall->perpendicularDistance(0.0f, 0.0f) <= STOP_FRONT_WALL_DISTANCE && elapsed >= STOP_DELAY)
    {
        stopTimerActive = false;

        state.robotMode = Mode::STOP;

        stateReturn.instantUpdate = true;
        return stateReturn;
    }

    return stateReturn;
}

void update(float dt, LidarModule &lidar, Pico2Module &pico2, State &state, float &outMotorSpeed, float &outSteeringPercent) {

    auto robotDataOpt = updateRobotData(dt, lidar, pico2, state);
    if (not robotDataOpt.has_value()) return;
    RobotData robotData = robotDataOpt.value();

    StateReturn stateReturn;
    do {
        StateReturn newStateReturn;
        stateReturn = newStateReturn;

        switch (state.robotMode) {
        default:
            std::cout << "[OpenChallenge] Invalid Mode!" << std::endl;
            stop_flag = 1;
            return;
        case Mode::NORMAL:
            stateReturn = stateNormal(robotData, state, outMotorSpeed, outSteeringPercent);
            break;
        case Mode::PRE_TURN:
            stateReturn = statePreTurn(robotData, state, outMotorSpeed, outSteeringPercent);
            break;
        case Mode::TURNING:
            stateReturn = stateTurning(robotData, state, outMotorSpeed, outSteeringPercent);
            break;
        case Mode::PRE_STOP:
            stateReturn = statePreStop(robotData, state, outMotorSpeed, outSteeringPercent);
            break;
        case Mode::STOP:
            // std::cout << "[Mode::STOP]\n";

            outMotorSpeed = 0.0f;
            outSteeringPercent = 0.0f;

            stop_flag = 1;
            return;
        }
    } while (stateReturn.instantUpdate);

    if (stateReturn.skipPid) return;

    float wallError = 0.0f;
    if (robotData.outerWall) {
        wallError = robotData.outerWall->perpendicularDistance(0.0f, 0.0f) - TARGET_OUTER_WALL_DISTANCE;
    }
    float headingErrorOffset = state.wallPid.update(wallError, dt);

    float headingError = state.headingDirection.toHeading() - robotData.heading;
    headingError = std::fmod(headingError + 180.0f, 360.0f);
    if (headingError < 0) headingError += 360.0f;
    headingError -= 180.0f;

    if (not stateReturn.skipWallPid) {
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

    auto dir = state.robotTurnDirection.value_or(RotationDirection::CLOCKWISE);
    // std::cout << "robotTurnDirection: " << (dir == RotationDirection::CLOCKWISE ? "CLOCKWISE" : "COUNTERCLOCKWISE") << "\n";
}

int main() {
    std::signal(SIGINT, signalHandler);

    const char *home = std::getenv("HOME");
    if (!home) throw std::runtime_error("HOME environment variable not set");
    std::string logFolder = std::string(home) + "/gfm_logs/open_challenge";

    std::string timedstampedLogFolder = Logger::generateTimestampedFolder(logFolder);

    Logger *lidarLogger = new Logger(timedstampedLogFolder + "/lidar.bin");
    Logger *pico2Logger = new Logger(timedstampedLogFolder + "/pico2.bin");
    Logger *openChallengeLogger = new Logger(timedstampedLogFolder + "/openChallenge.bin");

    // Initialize LidarModule
    LidarModule lidar(lidarLogger);
    if (!lidar.initialize()) return -1;
    lidar.printDeviceInfo();
    if (!lidar.start()) return -1;

    // Initialize Pico2Module
    Pico2Module pico2(pico2Logger);
    if (!pico2.initialize()) return -1;

    State state;

    if (wiringPiSetupGpio() == -1) {  // Use GPIO numbering
        printf("WiringPi setup failed.\n");
        return -1;
    }

    pinMode(BUTTON_PIN, INPUT);
    pullUpDnControl(BUTTON_PIN, PUD_UP);  // Enable pull-up resistor

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    while (digitalRead(BUTTON_PIN) == HIGH and !stop_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (!stop_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        lidar.startLogging();
        pico2.startLogging();

        const auto loopDuration = std::chrono::milliseconds(16);  // ~60 Hz
        auto lastTime = std::chrono::steady_clock::now();

        while (!stop_flag) {
            auto loopStart = std::chrono::steady_clock::now();

            std::chrono::duration<float> delta = loopStart - lastTime;
            float dt = delta.count();
            lastTime = loopStart;

            float motorSpeed, steeringPercent;
            update(dt, lidar, pico2, state, motorSpeed, steeringPercent);
            pico2.setMovementInfo(motorSpeed, steeringPercent);

            uint8_t dummyData[1] = {0x39};
            uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(loopStart.time_since_epoch()).count();
            openChallengeLogger->writeData(timestamp_ns, dummyData, sizeof(dummyData));

            // Maintain ~60 Hz loop rate
            auto loopEnd = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loopEnd - loopStart);
            if (elapsed < loopDuration) {
                std::this_thread::sleep_for(loopDuration - elapsed);
            }
        }
    } else {
        std::filesystem::remove_all(timedstampedLogFolder);
    }

    pico2.setMovementInfo(0.0f, 0.0f);

    // Shutdown
    lidar.stop();
    lidar.shutdown();
    pico2.shutdown();

    delete lidarLogger;
    delete pico2Logger;
    delete openChallengeLogger;

    std::cout << "[Main] Shutdown complete." << std::endl;
    return 0;
}

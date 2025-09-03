#include "camera_module.h"
#include "camera_processor.h"
#include "combined_processor.h"
#include "direction.h"
#include "lidar_module.h"
#include "lidar_processor.h"
#include "pico2_module.h"
#include "pid_controller.h"
#include "robot_pose_struct.h"

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

const uint32_t camWidth = 1296;
const uint32_t camHeight = 972;
const float camHFov = 104.0f;

const float TARGET_OUTER_WALL_DISTANCE = 0.50;
const float TARGET_OUTER_WALL_OUTER1_DISTANCE = 0.40;
const float TARGET_OUTER_WALL_OUTER2_DISTANCE = 0.22;
const float TARGET_OUTER_WALL_INNER1_DISTANCE = 0.62;
const float TARGET_OUTER_WALL_INNER2_DISTANCE = 0.78;
const float TARGET_OUTER_WALL_DISTANCE_PARKING = 0.30f;

const float PRE_TURN_FRONT_WALL_DISTANCE = 1.20f;
const auto PRE_TURN_COOLDOWN = std::chrono::milliseconds(1500);

const float TURNING_FRONT_WALL_DISTANCE = 0.80f;
const float TURNING_FRONT_WALL_OUTER1_DISTANCE = 0.70f;
const float TURNING_FRONT_WALL_OUTER2_DISTANCE = 0.50f;
const float TURNING_FRONT_WALL_INNER1_DISTANCE = 1.05f;
const float TURNING_FRONT_WALL_INNER2_DISTANCE = 1.13f;
const float TURNING_FRONT_WALL_CW_PARKING_DISTANCE = 0.64f;

// Will just go and park EZ
const float CCW_PRE_PARKING_FRONT_WALL_DISTANCE = 1.80f;
const auto CCW_PRE_FIND_PARKING_DELAY = std::chrono::milliseconds(500);
// Will just go to uturn then go and use CW_PRE_PARKING
const float CCW_UTURN_PRE_PARKING_FRONT_WALL_DISTANCE = 0.60f;
const auto CCW_UTURN_PRE_FIND_PARKING_DELAY = std::chrono::milliseconds(1000);
// Will go over then long reverse to park
const float CW_PRE_PARKING_FRONT_WALL_DISTANCE = 1.40f;
const auto CW_PRE_FIND_PARKING_DELAY = std::chrono::milliseconds(1000);
// Will just go to uturn then go and use CCW_PRE_PARKING
const float CW_UTURN_PRE_PARKING_FRONT_WALL_DISTANCE = 0.60f;
const auto CW_UTURN_PRE_FIND_PARKING_DELAY = std::chrono::milliseconds(1000);

enum Mode
{
    NORMAL,
    PRE_TURN,
    TURNING,
    CW_PRE_FIND_PARKING,
    CW_UTURN_PRE_FIND_PARKING_1,
    CW_UTURN_PRE_FIND_PARKING_2,
    CW_UTURN_PRE_FIND_PARKING_3,
    CCW_PRE_FIND_PARKING,
    CCW_UTURN_PRE_FIND_PARKING_1,
    CCW_UTURN_PRE_FIND_PARKING_2,
    CCW_UTURN_PRE_FIND_PARKING_3,
    CCW_FIND_PARKING,
    CW_FIND_PARKING,
    PARKING_1,
    PARKING_2,
    PARKING_3,
    STOP
};

struct State {
    PIDController headingPid{3.0f, 0.0, 0.0f};
    PIDController wallPid{180.0f, 0.0, 0.0f};

    std::map<std::pair<Segment, SegmentLocation>, std::vector<combined_processor::ClassifiedTrafficLight>> detectionHistory;
    std::map<std::pair<Segment, SegmentLocation>, combined_processor::ClassifiedTrafficLight> trafficLightMap;

    std::optional<float> initialHeading;
    std::optional<RotationDirection> robotTurnDirection;
    Mode robotMode = Mode::NORMAL;
    int numberOfTurn = 0;
    Direction headingDirection = Direction::NORTH;

    // NOTE: Parking COUNTER_CLOCKWISE no UTURN test
    // std::optional<float> initialHeading;
    // std::optional<RotationDirection> robotTurnDirection = RotationDirection::COUNTER_CLOCKWISE;
    // Mode robotMode = Mode::FIND_PARKING;
    // int numberOfTurn = 0;
    // Direction headingDirection = Direction::NORTH;
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
    // RobotDeltaPose deltaPose = {0.0f, 0.0f, 0.0f};

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

    auto trafficLightInfos = combined_processor::combineTrafficLightInfo(blockAngles, trafficLightPoints, deltaPose);

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

        if (state.robotMode != Mode::TURNING) {
            auto classifiedLights = combined_processor::classifyTrafficLights(
                trafficLightInfos,
                resolveWalls,
                *state.robotTurnDirection,
                Segment::fromDirection(state.headingDirection)
            );

            for (const auto &cl : classifiedLights) {
                std::pair<Segment, SegmentLocation> key = {cl.location.segment, cl.location.location};

                // Append to detection history
                auto &history = state.detectionHistory[key];
                history.push_back(cl);

                // Keep history limited (avoid unbounded growth)
                if (history.size() > 3) {
                    history.erase(history.begin());
                }

                // Check if we have 3 consecutive same wallSide and color
                if (history.size() == 3) {
                    bool allSame = true;
                    for (size_t i = 1; i < history.size(); ++i) {
                        if (history[i].location.side != history[0].location.side ||
                            history[i].info.cameraBlock.color != history[0].info.cameraBlock.color)
                        {
                            allSame = false;
                            break;
                        }
                    }

                    if (allSame && state.trafficLightMap.find(key) == state.trafficLightMap.end()) {
                        state.trafficLightMap[key] = history[0];  // Commit once

                        std::string colorStr;
                        switch (history[0].info.cameraBlock.color) {
                        case camera_processor::Color::RED:
                            colorStr = "RED";
                            break;
                        case camera_processor::Color::GREEN:
                            colorStr = "GREEN";
                            break;
                        default:
                            colorStr = "UNKNOWN";
                            break;
                        }

                        std::cout << "Traffic Light (" << colorStr << ") at LiDAR position (" << history[0].info.lidarPosition.x << ", "
                                  << history[0].info.lidarPosition.y << ")"
                                  << " mapped to Segment " << static_cast<int>(history[0].location.segment) << ", Location "
                                  << static_cast<int>(history[0].location.location) << ", WallSide "
                                  << (history[0].location.side == WallSide::INNER ? "INNER" : "OUTER")
                                  << " [3 consecutive matches confirmed]" << std::endl;
                    }
                }
            }
        }
        // std::cout << "Traffic Light Map contents:\n";
        // for (const auto &entry : state.trafficLightMap) {
        //     const auto &cl = entry.second;

        //     std::string colorStr;
        //     switch (cl.info.cameraBlock.color) {
        //     case camera_processor::Color::RED:
        //         colorStr = "RED";
        //         break;
        //     case camera_processor::Color::GREEN:
        //         colorStr = "GREEN";
        //         break;
        //     default:
        //         colorStr = "UNKNOWN";
        //         break;
        //     }

        //     std::cout << "Traffic Light (" << colorStr << ") at LiDAR position (" << cl.info.lidarPosition.x << ", "
        //               << cl.info.lidarPosition.y << ")"
        //               << " mapped to Segment " << static_cast<int>(cl.location.segment) << ", Location "
        //               << static_cast<int>(cl.location.location) << ", WallSide "
        //               << (cl.location.side == WallSide::INNER ? "INNER" : "OUTER") << std::endl;
        // }
    }

    bool pidWallErrorActive = true;

    static float targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE;
    float turningFrontWallDistance = TURNING_FRONT_WALL_DISTANCE;

    // TODO: Change this to match with obstacle_challenge
instant_update:
    switch (state.robotMode) {
    default:
        std::cout << "[ObstacleChallenge] Invalid Mode!" << std::endl;
        stop_flag = 1;
        return;

    case Mode::NORMAL: {
        // std::cout << "[Mode::NORMAL]\n";

        outMotorSpeed = 2.5f;

        Segment currentSegment = Segment::fromDirection(state.headingDirection);

        std::optional<combined_processor::ClassifiedTrafficLight> firstTrafficLight;
        std::optional<combined_processor::ClassifiedTrafficLight> secondTrafficLight;
        std::optional<combined_processor::ClassifiedTrafficLight> thirdTrafficLight;

        if (state.robotTurnDirection) {
            if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
                if (auto it = state.trafficLightMap.find({currentSegment, SegmentLocation::A}); it != state.trafficLightMap.end())
                    firstTrafficLight = it->second;
                if (auto it = state.trafficLightMap.find({currentSegment, SegmentLocation::B}); it != state.trafficLightMap.end())
                    secondTrafficLight = it->second;
                if (auto it = state.trafficLightMap.find({currentSegment, SegmentLocation::C}); it != state.trafficLightMap.end())
                    thirdTrafficLight = it->second;
            } else {  // COUNTER_CLOCKWISE
                if (auto it = state.trafficLightMap.find({currentSegment, SegmentLocation::C}); it != state.trafficLightMap.end())
                    firstTrafficLight = it->second;
                if (auto it = state.trafficLightMap.find({currentSegment, SegmentLocation::B}); it != state.trafficLightMap.end())
                    secondTrafficLight = it->second;
                if (auto it = state.trafficLightMap.find({currentSegment, SegmentLocation::A}); it != state.trafficLightMap.end())
                    thirdTrafficLight = it->second;
            }

            if (state.numberOfTurn == 12) {
                if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
                    if (firstTrafficLight && firstTrafficLight->info.cameraBlock.color == camera_processor::Color::RED) {
                        state.robotMode = Mode::CW_UTURN_PRE_FIND_PARKING_1;
                        goto instant_update;
                    } else {
                        state.robotMode = Mode::CW_PRE_FIND_PARKING;
                        goto instant_update;
                    }
                } else {  // COUNTER_CLOCKWISE
                    bool foundRed = false;
                    bool foundGreen = false;

                    if (firstTrafficLight) {
                        if (firstTrafficLight->info.cameraBlock.color == camera_processor::Color::RED) foundRed = true;
                        if (firstTrafficLight->info.cameraBlock.color == camera_processor::Color::GREEN) foundGreen = true;
                    }
                    if (secondTrafficLight) {
                        if (secondTrafficLight->info.cameraBlock.color == camera_processor::Color::RED) foundRed = true;
                        if (secondTrafficLight->info.cameraBlock.color == camera_processor::Color::GREEN) foundGreen = true;
                    }

                    if (foundRed) {
                        state.robotMode = Mode::CCW_PRE_FIND_PARKING;
                        goto instant_update;
                    } else if (foundGreen) {
                        state.robotMode = Mode::CCW_UTURN_PRE_FIND_PARKING_1;
                        goto instant_update;
                    }
                }
            }
        }

        float frontWallDistance = 0.0f;
        if (frontWall) {
            frontWallDistance = frontWall->perpendicularDistance(0.0f, 0.0f);
        } else if (backWall) {
            frontWallDistance = 3.0f - backWall->perpendicularDistance(0.0f, 0.0f);
        } else {
            frontWallDistance = 0.0f;
        }

        std::optional<combined_processor::ClassifiedTrafficLight> targetedTrafficLight;
        if (frontWallDistance > 2.00f && frontWallDistance <= 2.90f && firstTrafficLight) {
            targetedTrafficLight = firstTrafficLight;
        }
        if (frontWallDistance > 1.50f && frontWallDistance <= 2.40f && secondTrafficLight) {
            targetedTrafficLight = secondTrafficLight;
        }
        if (frontWallDistance > 1.00f && frontWallDistance <= 1.90f && thirdTrafficLight) {
            targetedTrafficLight = thirdTrafficLight;
        }

        if (targetedTrafficLight) {
            const auto &tl = *targetedTrafficLight;

            if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
                if (tl.info.cameraBlock.color == camera_processor::Color::GREEN) {
                    if (tl.location.side == WallSide::INNER) {
                        targetOuterWallDistance = TARGET_OUTER_WALL_OUTER1_DISTANCE;
                    } else {
                        targetOuterWallDistance = TARGET_OUTER_WALL_OUTER2_DISTANCE;
                    }
                } else if (tl.info.cameraBlock.color == camera_processor::Color::RED) {
                    if (tl.location.side == WallSide::INNER) {
                        targetOuterWallDistance = TARGET_OUTER_WALL_INNER2_DISTANCE;
                    } else {
                        targetOuterWallDistance = TARGET_OUTER_WALL_INNER1_DISTANCE;
                    }
                }
            } else {  // COUNTER_CLOCKWISE
                if (tl.info.cameraBlock.color == camera_processor::Color::GREEN) {
                    if (tl.location.side == WallSide::INNER) {
                        targetOuterWallDistance = TARGET_OUTER_WALL_INNER2_DISTANCE;
                    } else {
                        targetOuterWallDistance = TARGET_OUTER_WALL_INNER1_DISTANCE;
                    }
                } else if (tl.info.cameraBlock.color == camera_processor::Color::RED) {
                    if (tl.location.side == WallSide::INNER) {
                        targetOuterWallDistance = TARGET_OUTER_WALL_OUTER1_DISTANCE;
                    } else {
                        targetOuterWallDistance = TARGET_OUTER_WALL_OUTER2_DISTANCE;
                    }
                }
            }
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

        if (state.robotTurnDirection) {
            // --- Compute next segment ---
            Segment nextSegment;
            if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
                float nextHeading = state.headingDirection.toHeading() + 90.0f;
                nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
                nextSegment = Segment::fromHeading(nextHeading);
            } else {
                float nextHeading = state.headingDirection.toHeading() - 90.0f;
                nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
                nextSegment = Segment::fromHeading(nextHeading);
            }

            // --- Only pick first traffic light for the next segment ---
            std::optional<combined_processor::ClassifiedTrafficLight> nextFirstTrafficLight;
            if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
                if (auto it = state.trafficLightMap.find({nextSegment, SegmentLocation::A}); it != state.trafficLightMap.end()) {
                    nextFirstTrafficLight = it->second;
                }
            } else {  // COUNTER_CLOCKWISE
                if (auto it = state.trafficLightMap.find({nextSegment, SegmentLocation::C}); it != state.trafficLightMap.end()) {
                    nextFirstTrafficLight = it->second;
                }
            }

            if (nextFirstTrafficLight) {
                const auto &tl = *nextFirstTrafficLight;
                bool isGreen = (tl.info.cameraBlock.color == camera_processor::Color::GREEN);
                bool isInner = (tl.location.side == WallSide::INNER);

                if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
                    if (isGreen) {
                        if (state.numberOfTurn == 11) {
                            turningFrontWallDistance = TURNING_FRONT_WALL_CW_PARKING_DISTANCE;
                        } else if (isInner) {
                            turningFrontWallDistance = TURNING_FRONT_WALL_OUTER1_DISTANCE;
                        } else {
                            turningFrontWallDistance = TURNING_FRONT_WALL_OUTER2_DISTANCE;
                        }
                    } else {  // RED
                        if (isInner) {
                            turningFrontWallDistance = TURNING_FRONT_WALL_INNER2_DISTANCE;
                        } else {
                            turningFrontWallDistance = TURNING_FRONT_WALL_INNER1_DISTANCE;
                        }
                    }
                } else {  // COUNTER_CLOCKWISE
                    if (isGreen) {
                        if (isInner) {
                            turningFrontWallDistance = TURNING_FRONT_WALL_INNER2_DISTANCE;
                        } else {
                            turningFrontWallDistance = TURNING_FRONT_WALL_INNER1_DISTANCE;
                        }
                    } else {  // RED
                        if (isInner) {
                            turningFrontWallDistance = TURNING_FRONT_WALL_OUTER1_DISTANCE;
                        } else {
                            turningFrontWallDistance = TURNING_FRONT_WALL_OUTER2_DISTANCE;
                        }
                    }
                }
            }
        }

        // std::cout << "turningFrontWallDistance: " << turningFrontWallDistance << std::endl;

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

        targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE;

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
    case Mode::CCW_PRE_FIND_PARKING: {
        outMotorSpeed = 2.5f;
        targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE_PARKING;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (frontWall && frontWall->perpendicularDistance(0.0f, 0.0f) <= CCW_PRE_PARKING_FRONT_WALL_DISTANCE &&
            elapsed >= CCW_PRE_FIND_PARKING_DELAY)
        {
            waitTimerActive = false;

            state.robotMode = Mode::CCW_FIND_PARKING;
            goto instant_update;
        }
        break;
    }
    // FIXME: NOT TESTED
    case Mode::CCW_UTURN_PRE_FIND_PARKING_1: {
        outMotorSpeed = 2.5f;
        targetOuterWallDistance = TARGET_OUTER_WALL_INNER2_DISTANCE;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (frontWall && frontWall->perpendicularDistance(0.0f, 0.0f) <= CCW_UTURN_PRE_PARKING_FRONT_WALL_DISTANCE &&
            elapsed >= CCW_UTURN_PRE_FIND_PARKING_DELAY)
        {
            waitTimerActive = false;

            float nextHeading = state.headingDirection.toHeading() + 90.0f;
            nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
            state.headingDirection = Direction::fromHeading(nextHeading);
            state.robotMode = Mode::CCW_UTURN_PRE_FIND_PARKING_2;
            goto instant_update;
        }
        break;
    }
    // FIXME: NOT TESTED
    case Mode::CCW_UTURN_PRE_FIND_PARKING_2: {
        outMotorSpeed = 2.5f;
        pidWallErrorActive = false;

        float diff = heading - state.headingDirection.toHeading();
        diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        if (std::abs(diff) <= 20.0f) {
            float nextHeading = state.headingDirection.toHeading() + 90.0f;
            nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
            state.headingDirection = Direction::fromHeading(nextHeading);
            state.robotMode = Mode::CCW_UTURN_PRE_FIND_PARKING_3;
            goto instant_update;
        }
        break;
    }
    // FIXME: NOT TESTED
    case Mode::CCW_UTURN_PRE_FIND_PARKING_3: {
        outMotorSpeed = 2.5f;
        pidWallErrorActive = false;

        float diff = heading - state.headingDirection.toHeading();
        diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        if (std::abs(diff) <= 20.0f) {
            state.robotTurnDirection = RotationDirection::CLOCKWISE;
            state.robotMode = Mode::CW_PRE_FIND_PARKING;
            goto instant_update;
        }
        break;
    }
    // FIXME: NOT TESTED
    case Mode::CW_PRE_FIND_PARKING: {
        outMotorSpeed = 2.5f;
        targetOuterWallDistance = TARGET_OUTER_WALL_DISTANCE_PARKING;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (frontWall && frontWall->perpendicularDistance(0.0f, 0.0f) <= CW_PRE_PARKING_FRONT_WALL_DISTANCE &&
            elapsed >= CW_PRE_FIND_PARKING_DELAY)
        {
            waitTimerActive = false;

            state.robotMode = Mode::CW_FIND_PARKING;
            goto instant_update;
        }
        break;
    }
    // FIXME: NOT TESTED
    case Mode::CW_UTURN_PRE_FIND_PARKING_1: {
        outMotorSpeed = 2.5f;
        targetOuterWallDistance = TARGET_OUTER_WALL_INNER2_DISTANCE;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (frontWall && frontWall->perpendicularDistance(0.0f, 0.0f) <= CW_UTURN_PRE_PARKING_FRONT_WALL_DISTANCE &&
            elapsed >= CW_UTURN_PRE_FIND_PARKING_DELAY)
        {
            waitTimerActive = false;

            float nextHeading = state.headingDirection.toHeading() - 90.0f;
            nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
            state.headingDirection = Direction::fromHeading(nextHeading);
            state.robotMode = Mode::CW_UTURN_PRE_FIND_PARKING_2;
            goto instant_update;
        }
        break;
    }
    // FIXME: NOT TESTED
    case Mode::CW_UTURN_PRE_FIND_PARKING_2: {
        outMotorSpeed = 2.5f;
        pidWallErrorActive = false;

        float diff = heading - state.headingDirection.toHeading();
        diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        if (std::abs(diff) <= 20.0f) {
            float nextHeading = state.headingDirection.toHeading() - 90.0f;
            nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
            state.headingDirection = Direction::fromHeading(nextHeading);
            state.robotMode = Mode::CW_UTURN_PRE_FIND_PARKING_3;
            goto instant_update;
        }
        break;
    }
    // FIXME: NOT TESTED
    case Mode::CW_UTURN_PRE_FIND_PARKING_3: {
        outMotorSpeed = 2.5f;
        pidWallErrorActive = false;

        float diff = heading - state.headingDirection.toHeading();
        diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        if (std::abs(diff) <= 20.0f) {
            state.robotTurnDirection = RotationDirection::CLOCKWISE;
            state.robotMode = Mode::CCW_PRE_FIND_PARKING;
            goto instant_update;
        }
        break;
    }
    case Mode::CCW_FIND_PARKING: {
        outMotorSpeed = 1.0f;

        auto lineSegmentsForParking = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        auto parkingWalls = lidar_processor::getParkingWalls(lineSegmentsForParking, state.headingDirection, heading, 0.30f);

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

        // std::cout << "[BackParkingWall] dir=" << backParkingWallDir << "°, dist=" << backParkingWallDist << " m" << std::endl;

        float targetParkingWallDistance = 0.475f;
        bool isBackParkingWallBehind = backParkingWallDir >= 240.0f && backParkingWallDir < 360.0f;

        if (isBackParkingWallBehind && backParkingWallDist >= targetParkingWallDistance) {
            state.robotMode = Mode::PARKING_1;
            goto instant_update;
        }

        break;
    }
    // FIXME: NOT TESTED
    case Mode::CW_FIND_PARKING: {
        outMotorSpeed = 0.0f;
        outSteeringPercent = 0.0f;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (elapsed < std::chrono::milliseconds(700)) return;

        outMotorSpeed = -1.0f;

        auto lineSegmentsForParking = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        auto parkingWalls = lidar_processor::getParkingWalls(lineSegmentsForParking, state.headingDirection, heading, 0.30f);

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

        // std::cout << "[BackParkingWall] dir=" << backParkingWallDir << "°, dist=" << backParkingWallDist << " m" << std::endl;

        float targetParkingWallDistance = 0.455f;
        bool isBackParkingWallBehind = backParkingWallDir >= 180.0f && backParkingWallDir < 300.0f;

        if (isBackParkingWallBehind && backParkingWallDist >= targetParkingWallDistance) {
            waitTimerActive = false;

            state.robotMode = Mode::PARKING_1;
            goto instant_update;
        }

        break;
    }
    case Mode::PARKING_1: {
        outMotorSpeed = 0.0f;
        outSteeringPercent = 0.0f;

        if (not state.robotTurnDirection) return;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (elapsed < std::chrono::milliseconds(300)) return;

        if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
            outSteeringPercent = -100.0f;
        } else {
            outSteeringPercent = 100.0f;
        }

        if (elapsed < std::chrono::milliseconds(600)) return;

        outMotorSpeed = -1.0f;

        static bool encoderStarted = false;
        static double startEncoderAngle = 0.0;

        if (!encoderStarted) {
            startEncoderAngle = timedPico2Data.encoderAngle;
            encoderStarted = true;
        }

        // std::cout << "[PARKING_1] encoderAngle=" << timedPico2Data.encoderAngle << " startEncoderAngle=" << startEncoderAngle
        //           << " delta=" << (timedPico2Data.encoderAngle - startEncoderAngle) << std::endl;

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

        if (not state.robotTurnDirection) return;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (elapsed < std::chrono::milliseconds(300)) return;

        if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
            outSteeringPercent = 100.0f;
        } else {
            outSteeringPercent = -100.0f;
        }

        if (elapsed < std::chrono::milliseconds(600)) return;

        outMotorSpeed = -1.0f;

        static bool encoderStarted = false;
        static double startEncoderAngle = 0.0;

        if (!encoderStarted) {
            startEncoderAngle = timedPico2Data.encoderAngle;
            encoderStarted = true;
        }

        // std::cout << "[PARKING_2] encoderAngle=" << timedPico2Data.encoderAngle << " startEncoderAngle=" << startEncoderAngle
        //           << " delta=" << (timedPico2Data.encoderAngle - startEncoderAngle) << std::endl;

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

        if (not state.robotTurnDirection) return;

        static bool waitTimerActive = false;
        static auto waitStartTime = std::chrono::steady_clock::now();
        if (!waitTimerActive) {
            waitTimerActive = true;
            waitStartTime = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - waitStartTime;
        if (elapsed < std::chrono::milliseconds(300)) return;

        if (*state.robotTurnDirection == RotationDirection::CLOCKWISE) {
            outSteeringPercent = -100.0f;
        } else {
            outSteeringPercent = 100.0f;
        }

        if (elapsed < std::chrono::milliseconds(600)) return;

        outMotorSpeed = 1.0f;

        static bool encoderStarted = false;
        static double startEncoderAngle = 0.0;

        if (!encoderStarted) {
            startEncoderAngle = timedPico2Data.encoderAngle;
            encoderStarted = true;
        }

        // std::cout << "[PARKING_3] encoderAngle=" << timedPico2Data.encoderAngle << " startEncoderAngle=" << startEncoderAngle
        //           << " delta=" << (timedPico2Data.encoderAngle - startEncoderAngle) << std::endl;

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

    // std::cout << "targetOuterWallDistance" << targetOuterWallDistance << std::endl;

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

    if (outMotorSpeed < 0) {
        outSteeringPercent = -outSteeringPercent;
    }

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
    std::string logFolder = std::string(home) + "/gfm_logs/obstacle_challenge";

    Logger *lidarLogger = new Logger(Logger::generateFilename(logFolder, "lidar"));
    Logger *pico2Logger = new Logger(Logger::generateFilename(logFolder, "pico2"));
    Logger *cameraLogger = new Logger(Logger::generateFilename(logFolder, "camera"));

    // Logger *lidarLogger = nullptr;
    // Logger *pico2Logger = nullptr;
    // Logger *cameraLogger = nullptr;

    // Initialize LidarModule
    LidarModule lidar(lidarLogger);
    if (!lidar.initialize()) return -1;
    lidar.printDeviceInfo();
    if (!lidar.start()) return -1;

    // Initialize Pico2Module
    Pico2Module pico2(pico2Logger);
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
    CameraModule camera(cameraLogger, cameraOptionCallback);
    if (!camera.start()) return -1;

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

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    const auto loopDuration = std::chrono::milliseconds(32);  // ~30 Hz
    auto lastTime = std::chrono::steady_clock::now();

    while (!stop_flag) {
        auto loopStart = std::chrono::steady_clock::now();

        std::chrono::duration<float> delta = loopStart - lastTime;
        float dt = delta.count();
        lastTime = loopStart;

        // std::cout << "dt: " << dt * 1000.0f << " ms" << std::endl;  // print in milliseconds

        float motorSpeed, steeringPercent;
        update(dt, lidar, pico2, camera, state, motorSpeed, steeringPercent);
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

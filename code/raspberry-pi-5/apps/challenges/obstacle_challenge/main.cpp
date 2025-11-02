#include "camera_module.h"
#include "camera_processor.h"
#include "combined_processor.h"
#include "direction.h"
#include "lidar_module.h"
#include "lidar_processor.h"
#include "pico2_module.h"
#include "pid_controller.h"
#include "robot_pose_struct.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <map>
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

const float TARGET_OUTER_WALL_DISTANCE = 0.50;
const float TARGET_OUTER_WALL_OUTER1_DISTANCE = 0.43;
const float TARGET_OUTER_WALL_OUTER2_DISTANCE = 0.25;
const float TARGET_OUTER_WALL_INNER1_DISTANCE = 0.62;
const float TARGET_OUTER_WALL_INNER2_DISTANCE = 0.76;
const float TARGET_OUTER_WALL_DISTANCE_PARKING_CCW = 0.31f;
const float TARGET_OUTER_WALL_DISTANCE_PARKING_CW = 0.31f;
const float TARGET_OUTER_WALL_UTURN_PARKING_DISTANCE_CCW = 0.75f;
const float TARGET_OUTER_WALL_UTURN_PARKING_DISTANCE_CW = 0.75f;

const float PRE_TURN_FRONT_WALL_DISTANCE = 1.20f;
const auto PRE_TURN_COOLDOWN = std::chrono::milliseconds(4000);

const float TURNING_FRONT_WALL_DISTANCE = 0.78f;
const float TURNING_FRONT_WALL_OUTER1_DISTANCE = 0.67f;
const float TURNING_FRONT_WALL_OUTER2_DISTANCE = 0.50f;
const float TURNING_FRONT_WALL_INNER1_DISTANCE = 0.97f;
const float TURNING_FRONT_WALL_INNER2_DISTANCE = 1.07f;
const float TURNING_FRONT_WALL_CCW_PARKING_DISTANCE = 0.60f;
const float TURNING_FRONT_WALL_CW_PARKING_DISTANCE = 0.59f;

const auto PRE_TURN_COOLDOWN_PUSH = std::chrono::milliseconds(2000);

const float TURNING_FRONT_WALL_DISTANCE_PUSH = 0.89f;
const float TURNING_FRONT_WALL_OUTER1_DISTANCE_PUSH = 0.76f;
const float TURNING_FRONT_WALL_OUTER2_DISTANCE_PUSH = 0.59f;
const float TURNING_FRONT_WALL_INNER1_DISTANCE_PUSH = 1.06f;
const float TURNING_FRONT_WALL_INNER2_DISTANCE_PUSH = 1.16f;

// Will just go and park EZ
const float CCW_PRE_PARKING_FRONT_WALL_DISTANCE = 1.20f;
const auto CCW_PRE_FIND_PARKING_DELAY = std::chrono::milliseconds(500);
// Will just go to uturn then go and use CW_PRE_PARKING
const float CCW_UTURN_PRE_PARKING_FRONT_WALL_DISTANCE = 0.80f;
const auto CCW_UTURN_PRE_FIND_PARKING_DELAY = std::chrono::milliseconds(1000);
// Will go over then reverse the go over again to make sure that the car aligned
const float CW_PRE_PARKING_FRONT_WALL_DISTANCE = 1.80f;
const auto CW_PRE_FIND_PARKING_DELAY_1 = std::chrono::milliseconds(1000);
const auto CW_PRE_FIND_PARKING_DELAY_2 = std::chrono::milliseconds(3000);
// Will just go to uturn then go and use CCW_PRE_PARKING
const float CW_UTURN_PRE_PARKING_FRONT_WALL_DISTANCE = 1.00f;
const auto CW_UTURN_PRE_FIND_PARKING_DELAY = std::chrono::milliseconds(1000);

const float FORWARD_MOTOR_SPEED = 3.0f;
const float FORWARD_MOTOR_SPEED_PUSH = 4.5f;
const float HEADING_TOLERANCE_DEGREES_TURN = 30.0f;
const float HEADING_TOLERANCE_DEGREES_UTURN = 40.0f;

// PID Gains
const double HEADING_PID_P = 3.0;
const double HEADING_PID_I = 0.0;
const double HEADING_PID_D = 0.0;
const double WALL_PID_P = 180.0;
const double WALL_PID_I = 0.0;
const double WALL_PID_D = 0.0;

// --- Helper Functions ---
float normalizeAngle(float angle) {
    angle = std::fmod(angle + 180.0f, 360.0f);
    if (angle < 0) angle += 360.0f;
    return angle - 180.0f;
}

// --- Main Robot Class ---

class Robot
{
public:
    enum class Mode
    {
        UNKNOWN,
        CCW_UNPARK_1,
        CCW_UNPARK_2,
        CCW_UNPARK_3,
        CCW_UNPARK_4,
        CW_UNPARK_1,
        CW_UNPARK_2,
        NORMAL,
        PRE_TURN,
        TURNING,
        CW_PRE_FIND_PARKING_1,
        CW_PRE_FIND_PARKING_2,
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

    Robot(LidarModule &lidar, Pico2Module &pico2, CameraModule &camera, Logger &obstacleChallengeLogger)
        : lidar_(lidar)
        , pico2_(pico2)
        , camera_(camera)
        , obstacleChallengeLogger_(obstacleChallengeLogger)
        , headingPid_(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D, -100.0, 100.0)
        , wallPid_(WALL_PID_P, WALL_PID_I, WALL_PID_D, -90.0, 90.0) {
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
            case Mode::UNKNOWN:
                std::cout << "[Mode::UNKNOWN]\n";
                instantUpdate = false;
                pico2_.setMovementInfo(0.0f, 0.0f);
                return;
            case Mode::CCW_UNPARK_1:
                std::cout << "[Mode::CCW_UNPARK_1]\n";
                instantUpdate = updateCcwUnpark1State(robotData);
                break;
            case Mode::CCW_UNPARK_2:
                std::cout << "[Mode::CCW_UNPARK_2]\n";
                instantUpdate = updateCcwUnpark2State(robotData);
                break;
            case Mode::CCW_UNPARK_3:
                std::cout << "[Mode::CCW_UNPARK_3]\n";
                instantUpdate = updateCcwUnpark3State(robotData);
                break;
            case Mode::CCW_UNPARK_4:
                std::cout << "[Mode::CCW_UNPARK_4]\n";
                instantUpdate = updateCcwUnpark4State(robotData);
                break;
            case Mode::CW_UNPARK_1:
                std::cout << "[Mode::CW_UNPARK_1]\n";
                instantUpdate = updateCwUnpark1State(robotData);
                break;
            case Mode::CW_UNPARK_2:
                std::cout << "[Mode::CW_UNPARK_2]\n";
                instantUpdate = updateCwUnpark2State(robotData);
                break;
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
            case Mode::CW_PRE_FIND_PARKING_1:
                std::cout << "[Mode::CW_PRE_FIND_PARKING_1]\n";
                instantUpdate = updateCwPreFindParking1State(robotData);
                break;
            case Mode::CW_PRE_FIND_PARKING_2:
                std::cout << "[Mode::CW_PRE_FIND_PARKING_2]\n";
                instantUpdate = updateCwPreFindParking2State(robotData);
                break;
            case Mode::CW_UTURN_PRE_FIND_PARKING_1:
                std::cout << "[Mode::CW_UTURN_PRE_FIND_PARKING_1]\n";
                instantUpdate = updateCwUturnPreFindParking1State(robotData);
                break;
            case Mode::CW_UTURN_PRE_FIND_PARKING_2:
                std::cout << "[Mode::CW_UTURN_PRE_FIND_PARKING_2]\n";
                instantUpdate = updateCwUturnPreFindParking2State(robotData);
                break;
            case Mode::CW_UTURN_PRE_FIND_PARKING_3:
                std::cout << "[Mode::CW_UTURN_PRE_FIND_PARKING_3]\n";
                instantUpdate = updateCwUturnPreFindParking3State(robotData);
                break;
            case Mode::CCW_PRE_FIND_PARKING:
                std::cout << "[Mode::CCW_PRE_FIND_PARKING]\n";
                instantUpdate = updateCcwPreFindParkingState(robotData);
                break;
            case Mode::CCW_UTURN_PRE_FIND_PARKING_1:
                std::cout << "[Mode::CCW_UTURN_PRE_FIND_PARKING_1]\n";
                instantUpdate = updateCcwUturnPreFindParking1State(robotData);
                break;
            case Mode::CCW_UTURN_PRE_FIND_PARKING_2:
                std::cout << "[Mode::CCW_UTURN_PRE_FIND_PARKING_2]\n";
                instantUpdate = updateCcwUturnPreFindParking2State(robotData);
                break;
            case Mode::CCW_UTURN_PRE_FIND_PARKING_3:
                std::cout << "[Mode::CCW_UTURN_PRE_FIND_PARKING_3]\n";
                instantUpdate = updateCcwUturnPreFindParking3State(robotData);
                break;
            case Mode::CCW_FIND_PARKING:
                std::cout << "[Mode::CCW_FIND_PARKING]\n";
                instantUpdate = updateCcwFindParkingState(robotData);
                break;
            case Mode::CW_FIND_PARKING:
                std::cout << "[Mode::CW_FIND_PARKING]\n";
                instantUpdate = updateCwFindParkingState(robotData);
                break;
            case Mode::PARKING_1:
                std::cout << "[Mode::PARKING_1]\n";
                instantUpdate = updateParking1State(robotData);
                break;
            case Mode::PARKING_2:
                std::cout << "[Mode::PARKING_2]\n";
                instantUpdate = updateParking2State(robotData);
                break;
            case Mode::PARKING_3:
                std::cout << "[Mode::PARKING_3]\n";
                instantUpdate = updateParking3State(robotData);
                break;
            case Mode::STOP:
                motorSpeed_ = 0.0f;
                steeringPercent_ = 0.0f;
                stop_flag = 1;
                break;
            }
        } while (instantUpdate && !stop_flag);

        bool isUnparking =
            (mode_ == Mode::CCW_UNPARK_1 || mode_ == Mode::CCW_UNPARK_2 || mode_ == Mode::CCW_UNPARK_3 || mode_ == Mode::CCW_UNPARK_4 ||
             mode_ == Mode::CW_UNPARK_1 || mode_ == Mode::CW_UNPARK_2);
        bool isParking = (mode_ == Mode::PARKING_1 || mode_ == Mode::PARKING_2 || mode_ == Mode::PARKING_3);
        if (mode_ != Mode::STOP && !isParking && !isUnparking) {
            calculateSteering(dt, robotData);
        }

        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
    }

private:
    LidarModule &lidar_;
    Pico2Module &pico2_;
    CameraModule &camera_;
    Logger &obstacleChallengeLogger_;
    PIDController headingPid_;
    PIDController wallPid_;

    // --- State Variables ---
    Mode mode_ = Mode::UNKNOWN;
    Direction headingDirection_ = Direction::NORTH;
    std::optional<float> initialHeading_;
    std::optional<RotationDirection> turnDirection_;
    int turnCount_ = 0;

    // Mode mode_ = Mode::NORMAL;
    // Direction headingDirection_ = Direction::NORTH;
    // std::optional<float> initialHeading_;
    // std::optional<RotationDirection> turnDirection_ = RotationDirection::CLOCKWISE;
    // int turnCount_ = 12;

    // Mode mode_ = Mode::NORMAL;
    // Direction headingDirection_ = Direction::NORTH;
    // std::optional<float> initialHeading_;
    // std::optional<RotationDirection> turnDirection_ = RotationDirection::COUNTER_CLOCKWISE;
    // int turnCount_ = 12;

    // Mode mode_ = Mode::CCW_UNPARK_1;
    // Direction headingDirection_ = Direction::NORTH;
    // std::optional<float> initialHeading_;
    // std::optional<RotationDirection> turnDirection_ = RotationDirection::COUNTER_CLOCKWISE;
    // int turnCount_ = 0;

    // Mode mode_ = Mode::CW_UNPARK_1;
    // Direction headingDirection_ = Direction::NORTH;
    // std::optional<float> initialHeading_;
    // std::optional<RotationDirection> turnDirection_ = RotationDirection::CLOCKWISE;
    // int turnCount_ = 0;

    float targetOuterWallDistance_ = TARGET_OUTER_WALL_DISTANCE;
    float turningFrontWallDistance_ = TURNING_FRONT_WALL_DISTANCE;
    double startEncoderAngle_ = 0.0;

    std::map<std::pair<Segment, SegmentLocation>, std::vector<combined_processor::ClassifiedTrafficLight>> detectionHistory_;
    std::map<std::pair<Segment, SegmentLocation>, combined_processor::ClassifiedTrafficLight> trafficLightMap_;

    bool isNormalNoBlockWhenEnter = false;

    // Timers
    std::optional<std::chrono::steady_clock::time_point> lastPreTurnTimestamp_;
    std::optional<std::chrono::steady_clock::time_point> timer_;

    // Outputs
    float motorSpeed_ = 0.0f;
    float steeringPercent_ = 0.0f;

    struct RobotData {
        float heading;
        double encoderAngle;
        std::optional<lidar_processor::LineSegment> frontWall, backWall, outerWall, innerWall;
        std::vector<lidar_processor::LineSegment> parkingWalls;
    };

    // --- Method Implementations ---

    /**
     * @brief Calculates the recent rate of heading change in degrees per second.
     *
     * It compares the latest heading measurement with one from a fixed number of samples
     * in the past to determine the angular velocity.
     *
     * @param picoHistory The vector of recent pico2 data points.
     * @return The angular velocity in degrees per second.
     */
    float calculateRecentHeadingRate(const std::vector<TimedPico2Data> &picoHistory) {
        // The number of samples to look back for the calculation.
        // (size - 1) vs (size - 13) is a 12-sample difference.
        const size_t HEADING_RATE_LOOKBACK = 12;

        if (picoHistory.size() < HEADING_RATE_LOOKBACK + 1) {
            return 0.0f;  // Not enough data to compute
        }

        const auto &latestData = picoHistory.back();
        const auto &olderData = picoHistory[picoHistory.size() - 1 - HEADING_RATE_LOOKBACK];

        // Calculate the shortest angle difference (handles 360->0 wrap-around)
        float diff_deg = latestData.euler.h - olderData.euler.h;
        diff_deg = std::fmod(diff_deg + 180.0f, 360.0f) - 180.0f;

        // Use std::chrono for a type-safe and readable time difference calculation
        std::chrono::duration<float> timeElapsed = latestData.timestamp - olderData.timestamp;
        float timeElapsed_s = timeElapsed.count();

        // Avoid division by zero
        if (timeElapsed_s <= 0.0f) {
            return 0.0f;
        }

        return diff_deg / timeElapsed_s;
    }

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
        } obstacleChallengeData{
            static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timedLidarData.timestamp.time_since_epoch()).count()
            ),
            static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timedPico2Data.timestamp.time_since_epoch()).count()
            ),
            static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(timedFrame.timestamp.time_since_epoch()).count())
        };
        obstacleChallengeLogger_.writeData(timestamp_ns, &obstacleChallengeData, sizeof(obstacleChallengeData));

        if (!initialHeading_) {
            initialHeading_ = timedPico2Data.euler.h;
            return std::nullopt;
        }

        RobotData data;
        data.heading = timedPico2Data.euler.h - initialHeading_.value_or(0.0f);
        data.heading = std::fmod(data.heading, 360.0f);
        if (data.heading < 0.0f) data.heading += 360.0f;
        data.encoderAngle = timedPico2Data.encoderAngle;

        auto filteredLidarData = lidar_processor::filterLidarData(timedLidarData);
        auto deltaPose = combined_processor::aproximateRobotPose(filteredLidarData, timedPico2Datas);
        auto lineSegments = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        auto relativeWalls = lidar_processor::getRelativeWalls(lineSegments, headingDirection_, data.heading, 0.30f, 25.0f, 0.22f);
        auto resolvedWalls = lidar_processor::resolveWalls(relativeWalls);

        // TODO: Test this more extensively
        if (!turnDirection_) {
            auto unfilteredLineSegments = lidar_processor::getLines(timedLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
            auto unfilteredRelativeWalls =
                lidar_processor::getRelativeWalls(unfilteredLineSegments, headingDirection_, data.heading, 0.30f, 25.0f, 0.22f);
            turnDirection_ = lidar_processor::getTurnDirection(unfilteredRelativeWalls);
        }
        if (!turnDirection_) return std::nullopt;

        if (mode_ == Mode::UNKNOWN) {
            if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                mode_ = Mode::CW_UNPARK_1;
            } else {
                mode_ = Mode::CCW_UNPARK_1;
            }
        }

        data.frontWall = resolvedWalls.frontWall;
        data.backWall = resolvedWalls.backWall;
        if (turnDirection_) {
            data.outerWall = (*turnDirection_ == RotationDirection::CLOCKWISE) ? resolvedWalls.leftWall : resolvedWalls.rightWall;
            data.innerWall = (*turnDirection_ == RotationDirection::CLOCKWISE) ? resolvedWalls.rightWall : resolvedWalls.leftWall;
        }

        using namespace std::chrono_literals;
        float headingRate = calculateRecentHeadingRate(timedPico2Datas);
        // FIXME: Changing from 10.0f to 20.0f
        // if (mode_ != Mode::TURNING && turnDirection_ && abs(headingRate) <= 10.0f) {
        if (mode_ != Mode::TURNING && turnDirection_ && abs(headingRate) <= 20.0f) {
            auto trafficLightPoints = lidar_processor::getTrafficLightPoints(filteredLidarData, resolvedWalls, deltaPose, turnDirection_);
            auto colorMasks = camera_processor::filterColors(timedFrame);
            auto blockAngles = camera_processor::computeBlockAngles(colorMasks, CAM_WIDTH, CAM_HFOV);
            auto trafficLightInfos = combined_processor::combineTrafficLightInfo(blockAngles, trafficLightPoints);
            auto classifiedLights = combined_processor::classifyTrafficLights(
                trafficLightInfos,
                resolvedWalls,
                *turnDirection_,
                Segment::fromDirection(headingDirection_)
            );

            for (const auto &cl : classifiedLights) {
                // +++ DEBUG +++
                std::string clColorStr = (cl.info.cameraBlock.color == camera_processor::Color::RED)     ? "RED"
                                         : (cl.info.cameraBlock.color == camera_processor::Color::GREEN) ? "GREEN"
                                                                                                         : "UNKNOWN";
                std::string clSideStr = (cl.location.side == WallSide::INNER ? "INNER" : "OUTER");
                std::cout << "\n--- Processing cl: Seg=" << static_cast<int>(cl.location.segment)
                          << ", Loc=" << static_cast<int>(cl.location.location) << ", Side=" << clSideStr << ", Color=" << clColorStr
                          << std::endl;
                // +++++++++++++

                if (cl.location.segment == Segment::A && cl.location.side == WallSide::OUTER) continue;
                std::pair<Segment, SegmentLocation> key = {cl.location.segment, cl.location.location};
                auto &history = detectionHistory_[key];
                history.push_back(cl);

                if (history.size() > 3) history.erase(history.begin());

                // +++ DEBUG: This is the "cout history" part +++
                std::cout << "Current History (size=" << history.size() << "): [";
                for (const auto &item : history) {
                    std::string itemColor = (item.info.cameraBlock.color == camera_processor::Color::RED)     ? "R"
                                            : (item.info.cameraBlock.color == camera_processor::Color::GREEN) ? "G"
                                                                                                              : "U";
                    std::string itemSide = (item.location.side == WallSide::INNER ? "I" : "O");
                    std::cout << "{S:" << itemSide << ",C:" << itemColor << "} ";
                }
                std::cout << "]" << std::endl;
                // +++++++++++++++++++++++++++++++++++++++++++++++

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
                    if (allSame && trafficLightMap_.find(key) == trafficLightMap_.end()) {
                        trafficLightMap_[key] = history[0];  // Commit once
                    }
                }
            }
            std::cout << "Traffic Light Map contents:\n";
            for (const auto &entry : trafficLightMap_) {
                const auto &cl = entry.second;

                std::string colorStr;
                switch (cl.info.cameraBlock.color) {
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

                std::cout << "Traffic Light (" << colorStr << ") at LiDAR position (" << cl.info.lidarPosition.x << ", "
                          << cl.info.lidarPosition.y << ")"
                          << " mapped to Segment " << static_cast<int>(cl.location.segment) << ", Location "
                          << static_cast<int>(cl.location.location) << ", WallSide "
                          << (cl.location.side == WallSide::INNER ? "INNER" : "OUTER") << std::endl;
            }
        }

        if (mode_ == Mode::CW_FIND_PARKING || mode_ == Mode::CCW_FIND_PARKING) {
            auto lineSegmentsForParking = lidar_processor::getLines(filteredLidarData, deltaPose, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
            data.parkingWalls = lidar_processor::getParkingWalls(lineSegmentsForParking, headingDirection_, data.heading, 0.30f);
        }

        return data;
    }

    void calculateSteering(float dt, const RobotData &data) {
        float headingError = normalizeAngle(headingDirection_.toHeading() - data.heading);

        float wallError = 0.0f;
        if (data.outerWall) {
            wallError = data.outerWall->perpendicularDistance(0.0f, 0.0f) - targetOuterWallDistance_;
        }

        float headingErrorOffset = wallPid_.update(wallError, dt);

        if (motorSpeed_ < 0) headingErrorOffset = -headingErrorOffset;

        if (wallPid_.isActive()) {
            if (turnDirection_.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) {
                headingError -= headingErrorOffset;
            } else {
                headingError += headingErrorOffset;
            }
        }

        if (motorSpeed_ < 0) headingError = -headingError;

        steeringPercent_ = headingPid_.update(headingError, dt);
    }

    bool updateCcwUnpark1State(const RobotData &data) {
        if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
        motorSpeed_ = 1.5f;
        steeringPercent_ = -100.0f;
        float targetEncoderAngle = 400;
        if (data.encoderAngle - startEncoderAngle_ >= targetEncoderAngle) {
            startEncoderAngle_ = 0.0;
            mode_ = Mode::CCW_UNPARK_2;
            return true;
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }

    bool updateCcwUnpark2State(const RobotData &data) {
        if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
        motorSpeed_ = 1.5f;
        steeringPercent_ = 100.0f;
        float targetEncoderAngle = 270;
        if (data.encoderAngle - startEncoderAngle_ >= targetEncoderAngle) {
            startEncoderAngle_ = 0.0;
            mode_ = Mode::CCW_UNPARK_3;
            return true;
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }

    bool updateCcwUnpark3State(const RobotData &data) {
        if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
        motorSpeed_ = 1.5f;
        steeringPercent_ = -100.0f;
        float targetEncoderAngle = 80;
        if (data.encoderAngle - startEncoderAngle_ >= targetEncoderAngle) {
            startEncoderAngle_ = 0.0;
            mode_ = Mode::CCW_UNPARK_4;
            return true;
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }

    bool updateCcwUnpark4State(const RobotData &data) {
        if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
        motorSpeed_ = 1.5f;
        steeringPercent_ = 0.0f;
        float targetEncoderAngle = 100;
        if (data.encoderAngle - startEncoderAngle_ >= targetEncoderAngle) {
            startEncoderAngle_ = 0.0;
            mode_ = Mode::NORMAL;
            return true;
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }

    bool updateCwUnpark1State(const RobotData &data) {
        if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
        motorSpeed_ = 1.5f;
        steeringPercent_ = 100.0f;
        float targetEncoderAngle = 450;
        if (data.encoderAngle - startEncoderAngle_ >= targetEncoderAngle) {
            startEncoderAngle_ = 0.0;
            mode_ = Mode::CW_UNPARK_2;
            return true;
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }

    bool updateCwUnpark2State(const RobotData &data) {
        if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
        motorSpeed_ = 1.5f;
        steeringPercent_ = -100.0f;
        float targetEncoderAngle = 350;
        if (data.encoderAngle - startEncoderAngle_ >= targetEncoderAngle) {
            startEncoderAngle_ = 0.0;
            mode_ = Mode::NORMAL;
            return true;
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }

    bool updateNormalState(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        auto preTurnCooldown = PRE_TURN_COOLDOWN;
        if (turnCount_ >= 5) {
            motorSpeed_ = FORWARD_MOTOR_SPEED_PUSH;
            preTurnCooldown = PRE_TURN_COOLDOWN_PUSH;
        }

        Segment currentSegment = Segment::fromDirection(headingDirection_);

        std::optional<combined_processor::ClassifiedTrafficLight> firstTrafficLight;
        std::optional<combined_processor::ClassifiedTrafficLight> secondTrafficLight;
        std::optional<combined_processor::ClassifiedTrafficLight> thirdTrafficLight;

        if (turnDirection_) {
            if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                if (auto it = trafficLightMap_.find({currentSegment, SegmentLocation::A}); it != trafficLightMap_.end())
                    firstTrafficLight = it->second;
                if (auto it = trafficLightMap_.find({currentSegment, SegmentLocation::B}); it != trafficLightMap_.end())
                    secondTrafficLight = it->second;
                if (auto it = trafficLightMap_.find({currentSegment, SegmentLocation::C}); it != trafficLightMap_.end())
                    thirdTrafficLight = it->second;
            } else {  // COUNTER_CLOCKWISE
                if (auto it = trafficLightMap_.find({currentSegment, SegmentLocation::C}); it != trafficLightMap_.end())
                    firstTrafficLight = it->second;
                if (auto it = trafficLightMap_.find({currentSegment, SegmentLocation::B}); it != trafficLightMap_.end())
                    secondTrafficLight = it->second;
                if (auto it = trafficLightMap_.find({currentSegment, SegmentLocation::A}); it != trafficLightMap_.end())
                    thirdTrafficLight = it->second;
            }

            if (turnCount_ == 12) {
                if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                    if (firstTrafficLight && firstTrafficLight->info.cameraBlock.color == camera_processor::Color::RED) {
                        mode_ = Mode::CW_UTURN_PRE_FIND_PARKING_1;
                        return true;
                    } else {
                        mode_ = Mode::CW_PRE_FIND_PARKING_1;
                        return true;
                    }
                } else {  // COUNTER_CLOCKWISE
                    bool foundRed = false;
                    bool foundGreen = false;

                    if (firstTrafficLight) {
                        if (firstTrafficLight->info.cameraBlock.color == camera_processor::Color::RED) foundRed = true;
                        if (firstTrafficLight->info.cameraBlock.color == camera_processor::Color::GREEN) foundGreen = true;
                    }

                    if (foundRed) {
                        mode_ = Mode::CCW_PRE_FIND_PARKING;
                        return true;
                    } else if (foundGreen) {
                        mode_ = Mode::CCW_UTURN_PRE_FIND_PARKING_1;
                        return true;
                    } else {
                        mode_ = Mode::CCW_PRE_FIND_PARKING;
                        return true;
                    }
                }
            }
        }

        if (!isNormalNoBlockWhenEnter) {
            isNormalNoBlockWhenEnter = ((!firstTrafficLight) && (!secondTrafficLight) && (!thirdTrafficLight));
        } else {
            wallPid_.setGains(300.0f, WALL_PID_I, WALL_PID_D);
        }

        float frontWallDistance = 0.0f;
        if (data.frontWall) {
            frontWallDistance = data.frontWall->perpendicularDistance(0.0f, 0.0f);
        } else if (data.backWall) {
            frontWallDistance = 3.0f - data.backWall->perpendicularDistance(0.0f, 0.0f);
        }

        std::optional<combined_processor::ClassifiedTrafficLight> targetedTrafficLight;
        if (frontWallDistance > 2.00f && frontWallDistance <= 2.90f && firstTrafficLight) {
            targetedTrafficLight = firstTrafficLight;
        }
        // FIXME: This maybe the issue i changed from 2.30f to 2.70f
        // if (frontWallDistance > 1.50f && frontWallDistance <= 2.30f && secondTrafficLight) {
        if (frontWallDistance > 1.50f && frontWallDistance <= 2.70f && secondTrafficLight) {
            targetedTrafficLight = secondTrafficLight;
        }
        if (frontWallDistance > 1.00f && frontWallDistance <= 1.80f && thirdTrafficLight) {
            targetedTrafficLight = thirdTrafficLight;
        }

        if (targetedTrafficLight && turnDirection_) {
            const auto &tl = *targetedTrafficLight;

            if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                if (tl.info.cameraBlock.color == camera_processor::Color::GREEN) {
                    targetOuterWallDistance_ =
                        (tl.location.side == WallSide::INNER) ? TARGET_OUTER_WALL_OUTER1_DISTANCE : TARGET_OUTER_WALL_OUTER2_DISTANCE;
                } else if (tl.info.cameraBlock.color == camera_processor::Color::RED) {
                    targetOuterWallDistance_ =
                        (tl.location.side == WallSide::INNER) ? TARGET_OUTER_WALL_INNER2_DISTANCE : TARGET_OUTER_WALL_INNER1_DISTANCE;
                }
            } else {  // COUNTER_CLOCKWISE
                if (tl.info.cameraBlock.color == camera_processor::Color::GREEN) {
                    targetOuterWallDistance_ =
                        (tl.location.side == WallSide::INNER) ? TARGET_OUTER_WALL_INNER2_DISTANCE : TARGET_OUTER_WALL_INNER1_DISTANCE;
                } else if (tl.info.cameraBlock.color == camera_processor::Color::RED) {
                    targetOuterWallDistance_ =
                        (tl.location.side == WallSide::INNER) ? TARGET_OUTER_WALL_OUTER1_DISTANCE : TARGET_OUTER_WALL_OUTER2_DISTANCE;
                }
            }
        }

        if (turnCount_ == 0) {
            targetOuterWallDistance_ = TARGET_OUTER_WALL_OUTER1_DISTANCE;
        }

        auto now = std::chrono::steady_clock::now();
        bool cooldownOver = !lastPreTurnTimestamp_ || (now - *lastPreTurnTimestamp_ >= preTurnCooldown);

        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= PRE_TURN_FRONT_WALL_DISTANCE && cooldownOver) {
            if (isNormalNoBlockWhenEnter) {
                wallPid_.setGains(WALL_PID_P, WALL_PID_I, WALL_PID_D);
                isNormalNoBlockWhenEnter = false;
            }

            mode_ = Mode::PRE_TURN;
            lastPreTurnTimestamp_ = now;
            return true;
        }

        return false;
    }

    bool updatePreTurnState(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        turningFrontWallDistance_ = TURNING_FRONT_WALL_DISTANCE;
        if (turnCount_ >= 4 && turnCount_ < 11) {
            motorSpeed_ = FORWARD_MOTOR_SPEED_PUSH;
            turningFrontWallDistance_ = TURNING_FRONT_WALL_DISTANCE_PUSH;
        }

        if (turnDirection_) {
            // --- Compute next segment ---
            Segment nextSegment;
            if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                float nextHeading = headingDirection_.toHeading() + 90.0f;
                nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
                nextSegment = Segment::fromHeading(nextHeading);
            } else {
                float nextHeading = headingDirection_.toHeading() - 90.0f;
                nextHeading = std::fmod(nextHeading + 360.0f, 360.0f);
                nextSegment = Segment::fromHeading(nextHeading);
            }

            // --- Only pick first traffic light for the next segment ---
            std::optional<combined_processor::ClassifiedTrafficLight> nextFirstTrafficLight;
            bool isMid = false;
            if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                if (auto it = trafficLightMap_.find({nextSegment, SegmentLocation::A}); it != trafficLightMap_.end()) {
                    nextFirstTrafficLight = it->second;
                } else if (auto it = trafficLightMap_.find({nextSegment, SegmentLocation::B}); it != trafficLightMap_.end()) {
                    nextFirstTrafficLight = it->second;
                    isMid = true;
                }
            } else {  // COUNTER_CLOCKWISE
                if (auto it = trafficLightMap_.find({nextSegment, SegmentLocation::C}); it != trafficLightMap_.end()) {
                    nextFirstTrafficLight = it->second;
                } else if (auto it = trafficLightMap_.find({nextSegment, SegmentLocation::B}); it != trafficLightMap_.end()) {
                    nextFirstTrafficLight = it->second;
                    isMid = true;
                }
            }

            // FIXME: Fix this logic
            if (isMid && turnCount_ == 11) {
                if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                    turningFrontWallDistance_ = TURNING_FRONT_WALL_CW_PARKING_DISTANCE;
                } else {
                    turningFrontWallDistance_ = TURNING_FRONT_WALL_CCW_PARKING_DISTANCE;
                }
            } else if (nextFirstTrafficLight) {
                const auto &tl = *nextFirstTrafficLight;
                bool isGreen = (tl.info.cameraBlock.color == camera_processor::Color::GREEN);
                bool isInner = (tl.location.side == WallSide::INNER);

                if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                    if (isGreen) {
                        if (turnCount_ == 11) {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_CW_PARKING_DISTANCE;
                        } else if (isInner) {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_OUTER1_DISTANCE;
                            if (turnCount_ >= 4 && turnCount_ < 11) turningFrontWallDistance_ = TURNING_FRONT_WALL_OUTER1_DISTANCE_PUSH;
                        } else {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_OUTER2_DISTANCE;
                            if (turnCount_ >= 4 && turnCount_ < 11) turningFrontWallDistance_ = TURNING_FRONT_WALL_OUTER2_DISTANCE_PUSH;
                        }
                    } else {  // RED
                        if (isInner) {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_INNER2_DISTANCE;
                            if (turnCount_ >= 4 && turnCount_ < 11) turningFrontWallDistance_ = TURNING_FRONT_WALL_INNER2_DISTANCE_PUSH;
                        } else {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_INNER1_DISTANCE;
                            if (turnCount_ >= 4 && turnCount_ < 11) turningFrontWallDistance_ = TURNING_FRONT_WALL_INNER1_DISTANCE_PUSH;
                        }
                    }
                } else {  // COUNTER_CLOCKWISE
                    if (isGreen) {
                        if (isInner) {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_INNER2_DISTANCE;
                            if (turnCount_ >= 4 && turnCount_ < 11) turningFrontWallDistance_ = TURNING_FRONT_WALL_INNER2_DISTANCE_PUSH;
                        } else {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_INNER1_DISTANCE;
                            if (turnCount_ >= 4 && turnCount_ < 11) turningFrontWallDistance_ = TURNING_FRONT_WALL_INNER1_DISTANCE_PUSH;
                        }
                    } else {  // RED
                        if (turnCount_ == 11) {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_CCW_PARKING_DISTANCE;
                        } else if (isInner) {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_OUTER1_DISTANCE;
                            if (turnCount_ >= 4 && turnCount_ < 11) turningFrontWallDistance_ = TURNING_FRONT_WALL_OUTER1_DISTANCE_PUSH;
                        } else {
                            turningFrontWallDistance_ = TURNING_FRONT_WALL_OUTER2_DISTANCE;
                            if (turnCount_ >= 4 && turnCount_ < 11) turningFrontWallDistance_ = TURNING_FRONT_WALL_OUTER2_DISTANCE_PUSH;
                        }
                    }
                }
            } else {
                if (turnCount_ == 11) {
                    if (*turnDirection_ == RotationDirection::CLOCKWISE) {
                        turningFrontWallDistance_ = TURNING_FRONT_WALL_CW_PARKING_DISTANCE;
                    } else {
                        turningFrontWallDistance_ = TURNING_FRONT_WALL_CCW_PARKING_DISTANCE;
                    }
                }
            }
        }

        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= turningFrontWallDistance_) {
            float turnAngle = (turnDirection_.value_or(RotationDirection::CLOCKWISE) == RotationDirection::CLOCKWISE) ? 90.0f : -90.0f;
            float nextHeadingValue = std::fmod(headingDirection_.toHeading() + turnAngle + 360.0f, 360.0f);
            headingDirection_ = Direction::fromHeading(nextHeadingValue);

            mode_ = Mode::TURNING;
            return true;
        }

        return false;
    }

    bool updateTurningState(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        if (turnCount_ >= 4 && turnCount_ < 11) motorSpeed_ = FORWARD_MOTOR_SPEED_PUSH;
        targetOuterWallDistance_ = TARGET_OUTER_WALL_DISTANCE;
        wallPid_.setActive(false);

        float diff = data.heading - headingDirection_.toHeading();
        diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        if (std::abs(diff) <= HEADING_TOLERANCE_DEGREES_TURN) {
            turnCount_++;
            mode_ = Mode::NORMAL;
            wallPid_.setActive(true);
            return true;
        }
        return false;
    }

    bool updateCcwPreFindParkingState(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        wallPid_.setGains(300.0, WALL_PID_I, WALL_PID_D);
        targetOuterWallDistance_ = TARGET_OUTER_WALL_DISTANCE_PARKING_CCW;

        if (!timer_) {
            timer_ = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - *timer_;
        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= CCW_PRE_PARKING_FRONT_WALL_DISTANCE &&
            elapsed >= CCW_PRE_FIND_PARKING_DELAY)
        {
            timer_.reset();
            wallPid_.setGains(WALL_PID_P, WALL_PID_I, WALL_PID_D);
            mode_ = Mode::CCW_FIND_PARKING;
            return true;
        }
        return false;
    }

    bool updateCcwUturnPreFindParking1State(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        targetOuterWallDistance_ = TARGET_OUTER_WALL_UTURN_PARKING_DISTANCE_CCW;

        if (!timer_) {
            timer_ = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - *timer_;
        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= CCW_UTURN_PRE_PARKING_FRONT_WALL_DISTANCE &&
            elapsed >= CCW_UTURN_PRE_FIND_PARKING_DELAY)
        {
            timer_.reset();
            float nextHeading = headingDirection_.toHeading() + 90.0f;
            headingDirection_ = Direction::fromHeading(std::fmod(nextHeading + 360.0f, 360.0f));
            mode_ = Mode::CCW_UTURN_PRE_FIND_PARKING_2;
            return true;
        }
        return false;
    }

    bool updateCcwUturnPreFindParking2State(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        headingPid_.setGains(6.0, HEADING_PID_I, HEADING_PID_D);
        wallPid_.setActive(false);

        float diff = data.heading - headingDirection_.toHeading();
        diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        if (std::abs(diff) <= HEADING_TOLERANCE_DEGREES_UTURN) {
            float nextHeading = headingDirection_.toHeading() + 90.0f;
            headingDirection_ = Direction::fromHeading(std::fmod(nextHeading + 360.0f, 360.0f));
            headingPid_.setGains(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D);
            mode_ = Mode::CCW_UTURN_PRE_FIND_PARKING_3;
            return true;
        }
        return false;
    }

    bool updateCcwUturnPreFindParking3State(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        headingPid_.setGains(6.0, HEADING_PID_I, HEADING_PID_D);
        wallPid_.setActive(false);

        // float diff = data.heading - headingDirection_.toHeading();
        // diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        // if (diff <= HEADING_TOLERANCE_DEGREES_UTURN) {
        if (data.heading >= 170.0f && data.heading <= 270.0f) {
            turnDirection_ = RotationDirection::CLOCKWISE;
            headingPid_.setGains(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D);
            mode_ = Mode::CW_PRE_FIND_PARKING_1;
            wallPid_.setActive(true);  // Re-enable for the next state
            return true;
        }
        return false;
    }

    bool updateCwPreFindParking1State(const RobotData &data) {
        motorSpeed_ = 1.5f;
        wallPid_.setGains(300.0, WALL_PID_I, WALL_PID_D);
        targetOuterWallDistance_ = TARGET_OUTER_WALL_DISTANCE_PARKING_CW;

        if (!timer_) {
            timer_ = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - *timer_;
        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= CW_PRE_PARKING_FRONT_WALL_DISTANCE &&
            elapsed >= CW_PRE_FIND_PARKING_DELAY_1)
        {
            timer_.reset();
            wallPid_.setGains(WALL_PID_P, WALL_PID_I, WALL_PID_D);
            // mode_ = Mode::CW_PRE_FIND_PARKING_2;
            mode_ = Mode::CW_FIND_PARKING;
            return true;
        }
        return false;
    }

    bool updateCwPreFindParking2State(const RobotData &data) {
        if (!timer_) {
            timer_ = std::chrono::steady_clock::now();
        }
        auto elapsed = std::chrono::steady_clock::now() - *timer_;

        if (elapsed < std::chrono::milliseconds(700)) {
            motorSpeed_ = 0.0f;
            steeringPercent_ = 0.0f;
        } else if (elapsed < CW_PRE_FIND_PARKING_DELAY_2) {
            motorSpeed_ = -1.5f;
            targetOuterWallDistance_ = TARGET_OUTER_WALL_DISTANCE_PARKING_CW;
        } else {
            // Stop briefly before next state
            motorSpeed_ = 0.0f;
            steeringPercent_ = 0.0f;
            if (elapsed >= CW_PRE_FIND_PARKING_DELAY_2 + std::chrono::milliseconds(700)) {
                timer_.reset();
                mode_ = Mode::CW_FIND_PARKING;
                return true;
            }
        }
        return false;
    }

    bool updateCwUturnPreFindParking1State(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        targetOuterWallDistance_ = TARGET_OUTER_WALL_UTURN_PARKING_DISTANCE_CW;

        if (!timer_) {
            timer_ = std::chrono::steady_clock::now();
        }

        auto elapsed = std::chrono::steady_clock::now() - *timer_;
        if (data.frontWall && data.frontWall->perpendicularDistance(0.0f, 0.0f) <= CW_UTURN_PRE_PARKING_FRONT_WALL_DISTANCE &&
            elapsed >= CW_UTURN_PRE_FIND_PARKING_DELAY)
        {
            timer_.reset();
            float nextHeading = headingDirection_.toHeading() - 90.0f;
            headingDirection_ = Direction::fromHeading(std::fmod(nextHeading + 360.0f, 360.0f));
            mode_ = Mode::CW_UTURN_PRE_FIND_PARKING_2;
            return true;
        }
        return false;
    }

    bool updateCwUturnPreFindParking2State(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        headingPid_.setGains(6.0, HEADING_PID_I, HEADING_PID_D);
        wallPid_.setActive(false);

        float diff = data.heading - headingDirection_.toHeading();
        diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        if (std::abs(diff) <= HEADING_TOLERANCE_DEGREES_UTURN) {
            float nextHeading = headingDirection_.toHeading() - 90.0f;
            headingDirection_ = Direction::fromHeading(std::fmod(nextHeading + 360.0f, 360.0f));
            headingPid_.setGains(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D);
            mode_ = Mode::CW_UTURN_PRE_FIND_PARKING_3;
            return true;
        }
        return false;
    }

    bool updateCwUturnPreFindParking3State(const RobotData &data) {
        motorSpeed_ = FORWARD_MOTOR_SPEED;
        headingPid_.setGains(6.0, HEADING_PID_I, HEADING_PID_D);
        wallPid_.setActive(false);

        // float diff = data.heading - headingDirection_.toHeading();
        // diff = std::fmod(diff + 180.0f, 360.0f) - 180.0f;
        // if (std::abs(diff) <= HEADING_TOLERANCE_DEGREES_UTURN) {
        if (data.heading <= 190.0f && data.heading >= 90.0f) {
            turnDirection_ = RotationDirection::COUNTER_CLOCKWISE;
            headingPid_.setGains(HEADING_PID_P, HEADING_PID_I, HEADING_PID_D);
            mode_ = Mode::CCW_PRE_FIND_PARKING;
            wallPid_.setActive(true);
            return true;
        }
        return false;
    }

    bool updateCcwFindParkingState(const RobotData &data) {
        motorSpeed_ = 1.0f;
        targetOuterWallDistance_ = TARGET_OUTER_WALL_DISTANCE_PARKING_CCW;
        wallPid_.setGains(300.0, WALL_PID_I, WALL_PID_D);

        // if (data.parkingWalls.empty()) return false;

        // lidar_processor::LineSegment backParkingWall;
        // std::vector<lidar_processor::LineSegment> frontWalls;
        // std::vector<lidar_processor::LineSegment> backWalls;

        // for (auto &wall : data.parkingWalls) {
        //     if (wall.perpendicularDistance(0.0f, 0.0f) >= 1.00f) continue;

        //     float dir = wall.perpendicularDirection(0.0f, 0.0f);
        //     if (dir >= 0.0f && dir < 180.0f) {
        //         frontWalls.push_back(wall);
        //     } else {
        //         backWalls.push_back(wall);
        //     }
        // }

        // if (!frontWalls.empty() && !backWalls.empty()) {
        //     // Rule 1: if wall in front exists, pick closest back wall
        //     backParkingWall = *std::min_element(backWalls.begin(), backWalls.end(), [](const auto &a, const auto &b) {
        //         return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
        //     });
        // } else if (!frontWalls.empty()) {
        //     // Rule 2: only front walls → pick closest front wall
        //     backParkingWall = *std::min_element(frontWalls.begin(), frontWalls.end(), [](const auto &a, const auto &b) {
        //         return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
        //     });
        // } else if (!backWalls.empty()) {
        //     // Rule 3: only back walls → pick furthest back wall
        //     backParkingWall = *std::max_element(backWalls.begin(), backWalls.end(), [](const auto &a, const auto &b) {
        //         return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
        //     });
        // } else {
        //     return false;
        // }

        // float backParkingWallDir = backParkingWall.perpendicularDirection(0.0f, 0.0f);
        // float backParkingWallDist = -backParkingWall.y2;
        // float targetParkingWallDistance = 0.470f;
        // bool isBackParkingWallBehind = backParkingWallDir >= 240.0f && backParkingWallDir < 360.0f;

        // if (isBackParkingWallBehind && backParkingWallDist >= targetParkingWallDistance) {
        //     mode_ = Mode::PARKING_1;
        //     return true;
        // }
        // return false;

        if (!data.frontWall.has_value()) return false;

        float frontWallDist = data.frontWall->perpendicularDistance(0.0f, 0.0f);
        float targetFrontWallDistance = 0.96f;

        if (frontWallDist <= targetFrontWallDistance) {
            wallPid_.setGains(WALL_PID_P, WALL_PID_I, WALL_PID_D);
            mode_ = Mode::PARKING_1;
            return true;
        }
        return false;
    }

    bool updateCwFindParkingState(const RobotData &data) {
        motorSpeed_ = 1.0f;
        targetOuterWallDistance_ = TARGET_OUTER_WALL_DISTANCE_PARKING_CW;
        wallPid_.setGains(300.0, WALL_PID_I, WALL_PID_D);

        // if (data.parkingWalls.empty()) return false;

        // lidar_processor::LineSegment backParkingWall;
        // std::vector<lidar_processor::LineSegment> frontWalls;
        // std::vector<lidar_processor::LineSegment> backWalls;

        // for (auto &wall : data.parkingWalls) {
        //     if (wall.perpendicularDistance(0.0f, 0.0f) >= 1.00f) continue;

        //     float dir = wall.perpendicularDirection(0.0f, 0.0f);
        //     if (dir >= 0.0f && dir < 180.0f) {
        //         frontWalls.push_back(wall);
        //     } else {
        //         backWalls.push_back(wall);
        //     }
        // }

        // if (!frontWalls.empty() && !backWalls.empty()) {
        //     // Rule 1: if wall in front exists, pick closest back wall
        //     backParkingWall = *std::min_element(backWalls.begin(), backWalls.end(), [](const auto &a, const auto &b) {
        //         return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
        //     });
        // } else if (!frontWalls.empty()) {
        //     // Rule 2: only front walls → pick closest front wall
        //     backParkingWall = *std::min_element(frontWalls.begin(), frontWalls.end(), [](const auto &a, const auto &b) {
        //         return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
        //     });
        // } else if (!backWalls.empty()) {
        //     // Rule 3: only back walls → pick furthest back wall
        //     backParkingWall = *std::max_element(backWalls.begin(), backWalls.end(), [](const auto &a, const auto &b) {
        //         return a.perpendicularDistance(0.0f, 0.0f) < b.perpendicularDistance(0.0f, 0.0f);
        //     });
        // } else {
        //     return false;
        // }

        // float backParkingWallDir = backParkingWall.perpendicularDirection(0.0f, 0.0f);
        // float backParkingWallDist = -backParkingWall.y2;
        // float targetParkingWallDistance = 0.400f;
        // bool isBackParkingWallBehind = backParkingWallDir >= 180.0f && backParkingWallDir < 300.0f;

        // if (isBackParkingWallBehind && backParkingWallDist >= targetParkingWallDistance) {
        //     mode_ = Mode::PARKING_1;
        //     return true;
        // }
        // return false;

        if (!data.frontWall.has_value()) return false;

        float frontWallDist = data.frontWall->perpendicularDistance(0.0f, 0.0f);
        float targetFrontWallDistance = 1.565f;

        if (frontWallDist <= targetFrontWallDistance) {
            wallPid_.setGains(WALL_PID_P, WALL_PID_I, WALL_PID_D);
            mode_ = Mode::PARKING_1;
            return true;
        }
        return false;
    }

    bool updateParking1State(const RobotData &data) {
        if (!timer_) timer_ = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::steady_clock::now() - *timer_;

        if (elapsed < std::chrono::milliseconds(300)) {
            motorSpeed_ = 0.0f;
            steeringPercent_ = 0.0f;
        } else if (elapsed < std::chrono::milliseconds(600)) {
            motorSpeed_ = 0.0f;
            steeringPercent_ = (*turnDirection_ == RotationDirection::CLOCKWISE) ? -100.0f : 100.0f;
        } else {
            if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
            motorSpeed_ = -1.0f;
            steeringPercent_ = (*turnDirection_ == RotationDirection::CLOCKWISE) ? -100.0f : 100.0f;
            float targetEncoderAngle = (*turnDirection_ == RotationDirection::CLOCKWISE) ? -460 : -460;
            if (data.encoderAngle - startEncoderAngle_ <= targetEncoderAngle) {
                timer_.reset();
                startEncoderAngle_ = 0.0;
                mode_ = Mode::PARKING_2;
                return true;
            }
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }

    bool updateParking2State(const RobotData &data) {
        if (!timer_) timer_ = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::steady_clock::now() - *timer_;

        if (elapsed < std::chrono::milliseconds(300)) {
            motorSpeed_ = 0.0f;
            steeringPercent_ = 0.0f;
        } else if (elapsed < std::chrono::milliseconds(600)) {
            motorSpeed_ = 0.0f;
            steeringPercent_ = (*turnDirection_ == RotationDirection::CLOCKWISE) ? 100.0f : -100.0f;
        } else {
            if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
            motorSpeed_ = -1.0f;
            steeringPercent_ = (*turnDirection_ == RotationDirection::CLOCKWISE) ? 100.0f : -100.0f;
            float targetEncoderAngle = (*turnDirection_ == RotationDirection::CLOCKWISE) ? -380 : -380;
            if (data.encoderAngle - startEncoderAngle_ <= targetEncoderAngle) {
                timer_.reset();
                startEncoderAngle_ = 0.0;
                mode_ = Mode::PARKING_3;
                return true;
            }
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }

    bool updateParking3State(const RobotData &data) {
        if (!timer_) timer_ = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::steady_clock::now() - *timer_;

        if (elapsed < std::chrono::milliseconds(300)) {
            motorSpeed_ = 0.0f;
            steeringPercent_ = 0.0f;
        } else if (elapsed < std::chrono::milliseconds(600)) {
            motorSpeed_ = 0.0f;
            steeringPercent_ = (*turnDirection_ == RotationDirection::CLOCKWISE) ? -100.0f : 100.0f;
        } else {
            if (startEncoderAngle_ == 0.0) startEncoderAngle_ = data.encoderAngle;
            motorSpeed_ = 1.0f;
            steeringPercent_ = (*turnDirection_ == RotationDirection::CLOCKWISE) ? -100.0f : 100.0f;
            float targetEncoderAngle = (*turnDirection_ == RotationDirection::CLOCKWISE) ? 80 : 80;
            if (data.encoderAngle - startEncoderAngle_ >= targetEncoderAngle) {
                timer_.reset();
                startEncoderAngle_ = 0.0;
                mode_ = Mode::STOP;
                return true;
            }
        }
        pico2_.setMovementInfo(motorSpeed_, steeringPercent_);
        return false;
    }
};

// --- Main Function ---

int main() {
    std::signal(SIGINT, signalHandler);

    const char *home = std::getenv("HOME");
    if (!home) {
        std::cerr << "HOME environment variable not set" << std::endl;
        return -1;
    }
    std::string logFolder = std::string(home) + "/gfm_logs/obstacle_challenge";
    std::string timedstampedLogFolder = Logger::generateTimestampedFolder(logFolder);

    Logger lidarLogger(timedstampedLogFolder + "/lidar.bin");
    Logger pico2Logger(timedstampedLogFolder + "/pico2.bin");
    Logger cameraLogger(timedstampedLogFolder + "/camera.bin");
    Logger obstacleChallengeLogger(timedstampedLogFolder + "/obstacleChallenge.bin");

    LidarModule lidar(&lidarLogger);
    Pico2Module pico2(&pico2Logger);
    CameraModule camera(&cameraLogger, cameraOptionCallback);

    if (!lidar.initialize() || !lidar.start()) {
        std::cerr << "LidarModule initialization failed." << std::endl;
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

    Robot robot(lidar, pico2, camera, obstacleChallengeLogger);

    if (wiringPiSetupGpio() == -1) {
        std::cerr << "WiringPi setup failed." << std::endl;
        return -1;
    }
    pinMode(BUTTON_PIN, INPUT);
    pullUpDnControl(BUTTON_PIN, PUD_UP);

    std::cout << "Press the button to start..." << std::endl;
    while (digitalRead(BUTTON_PIN) == HIGH && !stop_flag) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!stop_flag) {
        std::cout << "Starting in 1.0 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        lidar.startLogging();
        pico2.startLogging();
        camera.startLogging();

        const auto loopDuration = std::chrono::milliseconds(33);  // ~30 Hz
        auto lastTime = std::chrono::steady_clock::now();
        std::cout << "Robot running." << std::endl;
        while (!stop_flag) {
            auto loopStart = std::chrono::steady_clock::now();
            float dt = std::chrono::duration<float>(loopStart - lastTime).count();
            lastTime = loopStart;

            robot.update(dt);

            auto elapsed = std::chrono::steady_clock::now() - loopStart;
            if (elapsed < loopDuration) {
                std::this_thread::sleep_for(loopDuration - elapsed);
            }
        }
    } else {
        std::filesystem::remove_all(timedstampedLogFolder);
        std::cout << "Start aborted. Log folder removed." << std::endl;
    }

    std::cout << "Shutting down..." << std::endl;
    pico2.setMovementInfo(0.0f, 0.0f);
    lidar.stop();
    lidar.shutdown();
    pico2.shutdown();
    camera.stop();
    std::cout << "Shutdown complete." << std::endl;

    return 0;
}

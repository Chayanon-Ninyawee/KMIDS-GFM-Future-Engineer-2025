#include <cstring>
#include <iostream>
#include <vector>

#include "imu_struct.h"
#include "lidar_processor.h"
#include "lidar_struct.h"
#include "log_reader.h"

TimedLidarData reconstructTimedLidar(const LogEntry &entry) {
    std::vector<RawLidarNode> nodes(entry.data.size() / sizeof(RawLidarNode));
    if (!entry.data.empty()) {
        std::memcpy(nodes.data(), entry.data.data(), entry.data.size());
    }

    // Convert timestamp in nanoseconds back to steady_clock::time_point
    auto ts = std::chrono::nanoseconds(entry.timestamp);
    std::chrono::steady_clock::time_point timestamp(ts);

    return TimedLidarData{std::move(nodes), timestamp};
}

TimedImuData reconstructTimedImu(const LogEntry &entry) {
    TimedImuData imuData{};

    if (entry.data.size() < sizeof(imu_accel_float_t) + sizeof(imu_euler_float_t)) {
        throw std::runtime_error("Invalid IMU entry data size");
    }

    std::memcpy(&imuData.accel, entry.data.data(), sizeof(imu_accel_float_t));
    std::memcpy(&imuData.euler, entry.data.data() + sizeof(imu_accel_float_t), sizeof(imu_euler_float_t));

    // Convert timestamp in nanoseconds back to steady_clock::time_point
    auto ts = std::chrono::nanoseconds(entry.timestamp);
    imuData.timestamp = std::chrono::steady_clock::time_point(ts);

    return imuData;
}
float wrapHeading(float heading) {
    heading = std::fmod(heading, 360.0f);  // remainder after division
    if (heading < 0) {
        heading += 360.0f;  // keep it positive
    }
    return heading;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <lidar_log_file> <imu_log_file>" << std::endl;
        return 1;
    }

    std::string lidarLogFile = argv[1];
    LogReader lidarReader(lidarLogFile);
    std::vector<LogEntry> lidarEntries;

    if (!lidarReader.readAll(lidarEntries)) {
        std::cerr << "Failed to read log file." << std::endl;
        return 1;
    }

    std::cout << "Loaded " << lidarEntries.size() << " Lidar log entries." << std::endl;

    std::string imuLogFile = argv[2];
    LogReader imuReader(imuLogFile);
    std::vector<LogEntry> imuEntries;

    if (!imuReader.readAll(imuEntries)) {
        std::cerr << "Failed to read IMU log file." << std::endl;
        return 1;
    }

    std::cout << "Loaded " << imuEntries.size() << " IMU log entries." << std::endl;

    const auto &intialImuEntry = imuEntries[0];
    TimedImuData intialTimedImuData = reconstructTimedImu(intialImuEntry);

    cv::namedWindow("Video", cv::WINDOW_FULLSCREEN);

    size_t i = 0;
    while (true) {
        int key = cv::waitKey(0);  // waits for key press

        if (key == 27) break;  // ESC to exit

        if (key == 81) {  // left arrow
            if (i > 0) i--;
        } else if (key == 83) {  // right arrow
            if (i < lidarEntries.size() - 1) i++;
        } else {
            continue;
        }

        const auto &lidarEntry = lidarEntries[i];
        TimedLidarData timedLidarData = reconstructTimedLidar(lidarEntry);

        TimedLidarData filteredLidarData;
        filteredLidarData.timestamp = timedLidarData.timestamp;
        for (const auto &node : timedLidarData.lidarData) {
            if (node.distance >= 0.15f) {
                filteredLidarData.lidarData.push_back(node);
            }
        }

        const auto &imuEntry = imuEntries[i];
        TimedImuData timedImuData = reconstructTimedImu(imuEntry);
        timedImuData.euler.h -= intialTimedImuData.euler.h;
        timedImuData.euler.h = wrapHeading(timedImuData.euler.h);
        timedImuData.euler.r -= intialTimedImuData.euler.r;
        timedImuData.euler.r = wrapHeading(timedImuData.euler.h);
        timedImuData.euler.p -= intialTimedImuData.euler.p;
        timedImuData.euler.p = wrapHeading(timedImuData.euler.h);

        std::cout << timedImuData.euler.h << std::endl;

        auto lineSegments = lidar_processor::getLines(filteredLidarData, 0.05f, 10, 0.10f, 0.10f, 18.0f, 0.20f);
        // lineSegments = lidar_processor::getWalls(lineSegments, 0.30f, 25.0f, 0.22f);
        lineSegments = lidar_processor::getParkingWalls(lineSegments);

        cv::Mat lidarMat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
        lidar_processor::drawLidarData(lidarMat, timedLidarData);

        for (size_t i = 0; i < lineSegments.size(); ++i) {
            const auto &lineSegment = lineSegments[i];

            std::srand(static_cast<unsigned int>(i));
            cv::Scalar color(rand() % 256, rand() % 256, rand() % 256);

            // lidar_processor::drawLineSegment(lidarMat, lineSegment, 4.0f);
            lidar_processor::drawLineSegment(lidarMat, lineSegment, 4.0f, color);
        }

        cv::imshow("Video", lidarMat);
    }

    cv::destroyAllWindows();
    return 0;
}

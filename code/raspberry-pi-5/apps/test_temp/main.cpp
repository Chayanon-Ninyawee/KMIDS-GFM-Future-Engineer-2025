#include "lidar_module.h"

#include <chrono>
#include <csignal>
#include <iostream>

volatile std::sig_atomic_t stop_flag = 0;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    stop_flag = 1;
}

int main() {
    std::signal(SIGINT, signalHandler);

    LidarModule lidar;

    if (not lidar.initialize()) return -1;

    if (not lidar.start()) return -1;

    using clock = std::chrono::steady_clock;

    int frameCount = 0;
    auto startTime = clock::now();

    while (!stop_flag) {
        auto lidarRawData = lidar.getData();

        // for (const auto &node : lidarRawData) {
        //     printf("Angle: %.3f\tDistance: %.3f m\n", node.angle, node.distance);
        // }
        std::cout << "Node Count: " << lidarRawData.size() << std::endl;

        frameCount++;
        auto now = clock::now();
        std::chrono::duration<double> elapsed = now - startTime;

        if (elapsed.count() >= 1.0) {
            double hz = frameCount / elapsed.count();
            std::cout << "Loop frequency: " << hz << " Hz" << std::endl;

            // reset
            frameCount = 0;
            startTime = now;
        }
    }

    lidar.stop();
    lidar.shutdown();
}

#pragma once

#include <thread>

struct ImuAccel {
    float x, y, z;
};

struct ImuEuler {
    float h, r, p;
};

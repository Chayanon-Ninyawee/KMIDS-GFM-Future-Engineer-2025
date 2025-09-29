#pragma once

#include <cmath>
#include <cstdint>

/**
 * @brief Direction of the robot's turn or rotation.
 */
enum class RotationDirection
{
    CLOCKWISE,         ///< Robot or path rotates clockwise.
    COUNTER_CLOCKWISE  ///< Robot or path rotates counter-clockwise.
};

/**
 * @brief Relative side of a traffic light or object with respect to a reference wall.
 */
enum class WallSide
{
    INNER = 0,  ///< Closer to the inner wall.
    OUTER = 1   ///< Closer to the outer wall.
};

/**
 * @brief Relative side with respect to the robot or a generic reference frame.
 */
enum class RelativeSide
{
    LEFT = 0,   ///< Left side relative to robot or reference.
    RIGHT = 1,  ///< Right side relative to robot or reference.
    FRONT = 2,  ///< Front side relative to robot or reference.
    BACK = 3    ///< Back side relative to robot or reference.
};

/**
 * @brief Location within a segment, relative to front/mid/back along robot path.
 *
 * For clockwise (CW) rotation: A = front, B = mid, C = back.
 * For counter-clockwise (CCW) rotation: A = back, B = mid, C = front.
 */
enum class SegmentLocation
{
    A,  ///< Front (CW) or Back (CCW)
    B,  ///< Mid
    C   ///< Back (CW) or Front (CCW)
};

// Cardinal directions
class Direction
{
public:
    enum Value : uint8_t
    {
        NORTH = 0,
        EAST = 1,
        SOUTH = 2,
        WEST = 3
    };

    Direction() = default;
    constexpr Direction(Value dir)
        : value(dir) {}

    // Allow switch-case and comparisons
    constexpr operator Value() const {
        return value;
    }

    // Convert to heading in degrees
    constexpr float toHeading() const {
        switch (value) {
        case NORTH:
            return 0.0f;
        case EAST:
            return 90.0f;
        case SOUTH:
            return 180.0f;
        case WEST:
            return 270.0f;
        }
        return 0.0f;  // fallback
    }
    //
    // Construct a Direction from a heading in degrees
    static constexpr Direction fromHeading(float heading) {
        // Normalize to [0, 360)
        float h = std::fmod(heading + 360.0f, 360.0f);
        if (h >= 315.0f || h < 45.0f) return NORTH;
        if (h >= 45.0f && h < 135.0f) return EAST;
        if (h >= 135.0f && h < 225.0f) return SOUTH;
        return WEST;  // 225–315
    }

    /**
     * @brief Get the absolute Direction corresponding to a relative side.
     *
     * @param side RelativeSide (LEFT, RIGHT, FRONT, BACK)
     * @return Direction facing that side relative to the current direction
     */
    constexpr Direction fromRelativeSide(RelativeSide side) const {
        // Map relative side to a 0–3 offset (number of 90° rotations clockwise)
        uint8_t offset = 0;
        switch (side) {
        case RelativeSide::FRONT:
            offset = 0;
            break;
        case RelativeSide::RIGHT:
            offset = 1;
            break;
        case RelativeSide::BACK:
            offset = 2;
            break;
        case RelativeSide::LEFT:
            offset = 3;
            break;
        }
        return Direction(static_cast<Value>((static_cast<uint8_t>(value) + offset) % 4));
    }

private:
    Value value;
};

/**
 * @brief Quadrant segment of the robot's path or environment.
 *
 * The robot starts at Segment A, and numbering proceeds clockwise (CW) or
 * counter-clockwise (CCW) depending on RotationDirection.
 *
 * Example (robot facing north at start, segments clockwise):
 *
 *     North (0°)  → Segment A
 *     East  (90°) → Segment B
 *     South (180°)→ Segment C
 *     West  (270°)→ Segment D
 *
 * If rotation is counter-clockwise, the segment sequence becomes:
 *
 *     North (0°)  → Segment A
 *     West  (270°)→ Segment D
 *     South (180°)→ Segment C
 *     East  (90°) → Segment B
 */
class Segment
{
public:
    enum Value : uint8_t
    {
        A = 0,
        B = 1,
        C = 2,
        D = 3
    };

    Segment() = default;
    constexpr Segment(Value seg)
        : value(seg) {}

    // Allow switch-case and comparisons
    constexpr operator Value() const {
        return value;
    }

    /**
     * @brief Convert the Segment to a Direction object.
     */
    constexpr Direction toDirection() const {
        switch (value) {
        case A:
            return Direction(Direction::NORTH);
        case B:
            return Direction(Direction::EAST);
        case C:
            return Direction(Direction::SOUTH);
        case D:
            return Direction(Direction::WEST);
        }
        return Direction(Direction::NORTH);  // fallback
    }

    /**
     * @brief Convert the Segment to a heading in degrees.
     */
    constexpr float toHeading() const {
        return toDirection().toHeading();  // reuse Direction
    }

    /**
     * @brief Construct a Segment from a Direction object.
     */
    static constexpr Segment fromDirection(const Direction &dir) {
        switch (static_cast<Direction::Value>(dir)) {
        case Direction::NORTH:
            return A;
        case Direction::EAST:
            return B;
        case Direction::SOUTH:
            return C;
        case Direction::WEST:
            return D;
        }
        return A;  // fallback
    }

    /**
     * @brief Construct a Segment from a heading (degrees) via Direction.
     */
    static constexpr Segment fromHeading(float heading) {
        return fromDirection(Direction::fromHeading(heading));
    }

private:
    Value value;
};

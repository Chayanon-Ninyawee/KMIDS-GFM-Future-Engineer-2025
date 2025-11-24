## `direction.h` Reference: Robot and Path Orientation Concepts

This header defines various enums and classes for handling **direction, rotation, and segment location** in a robotic or path-planning context. It primarily deals with **cardinal directions** and related relative positions.

______________________________________________________________________

### Enums for Direction and Side

These enumerations define basic directional and side references.

#### `RotationDirection`

Describes the direction of a turn or rotation.

| Enum Value | Description |
| :--- | :--- |
| **`CLOCKWISE`** | Robot or path rotates clockwise. |
| **`COUNTER_CLOCKWISE`** | Robot or path rotates counter-clockwise. |

#### `WallSide`

Describes a relative side with respect to a reference wall, typically in a segment or corridor.

| Enum Value | Value | Description |
| :--- | :--- | :--- |
| **`INNER`** | 0 | Closer to the inner wall. |
| **`OUTER`** | 1 | Closer to the outer wall. |

#### `RelativeSide`

Describes a generic side relative to the robot's current orientation or a reference frame.

| Enum Value | Value | Description |
| :--- | :--- | :--- |
| **`LEFT`** | 0 | Left side relative to robot or reference. |
| **`RIGHT`** | 1 | Right side relative to robot or reference. |
| **`FRONT`** | 2 | Front side relative to robot or reference. |
| **`BACK`** | 3 | Back side relative to robot or reference. |

#### `SegmentLocation`

Describes a specific location within a smaller segment, relative to the front/mid/back of the path. The meaning of A and C depends on the rotation direction.

| Enum Value | Description |
| :--- | :--- |
| **`A`** | Front (**CW**) or Back (**CCW**) along the segment. |
| **`B`** | Midpoint along the segment. |
| **`C`** | Back (**CW**) or Front (**CCW**) along the segment. |

______________________________________________________________________

### `Direction` Class (Cardinal Directions)

The `Direction` class represents the four **cardinal directions** and provides utilities for conversion to/from headings (degrees) and relative sides.

#### `Direction::Value` (Cardinal Directions)

| Enum Value | Value | Heading (Degrees) |
| :--- | :--- | :--- |
| **`NORTH`** | 0 | $0.0^\\circ$ |
| **`EAST`** | 1 | $90.0^\\circ$ |
| **`SOUTH`** | 2 | $180.0^\\circ$ |
| **`WEST`** | 3 | $270.0^\\circ$ |

#### Key Methods

| Method | Description |
| :--- | :--- |
| `constexpr float toHeading() const` | Converts the direction to a heading in degrees. |
| `static constexpr Direction fromHeading(float heading)` | Constructs a `Direction` from a heading (degrees). Uses $45^\\circ$ boundaries (e.g., $315^\\circ$ to $< 45^\\circ$ is **NORTH**). |
| `constexpr Direction fromRelativeSide(RelativeSide side) const` | Calculates the absolute `Direction` (N, E, S, W) that corresponds to a **`RelativeSide`** (LEFT, RIGHT, FRONT, BACK) from the current direction. |

______________________________________________________________________

### `Segment` Class (Quadrant Path Segments)

The `Segment` class divides the environment or path into four major quadrants, often corresponding to the cardinal directions in a typical setup (e.g., North = A, East = B, etc.).

#### `Segment::Value` (Quadrant Segments)

| Enum Value | Value | Direction Equivalent (Default CW) | Heading (Degrees) |
| :--- | :--- | :--- | :--- |
| **`A`** | 0 | NORTH | $0.0^\\circ$ |
| **`B`** | 1 | EAST | $90.0^\\circ$ |
| **`C`** | 2 | SOUTH | $180.0^\\circ$ |
| **`D`** | 3 | WEST | $270.0^\\circ$ |

#### Key Methods

| Method | Description |
| :--- | :--- |
| `constexpr Direction toDirection() const` | Converts the `Segment` to its corresponding cardinal **`Direction`** (A $\\to$ NORTH, B $\\to$ EAST, etc.). |
| `constexpr float toHeading() const` | Converts the `Segment` to a heading in degrees by using `toDirection()`. |
| `static constexpr Segment fromDirection(const Direction &dir)` | Constructs a `Segment` from a cardinal **`Direction`**. |
| `static constexpr Segment fromHeading(float heading)` | Constructs a `Segment` from a heading (degrees) using the `Direction` conversion logic. |

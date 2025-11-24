## Actuation Components Reference (Microcontroller)

This section provides the API reference for the low-level motor and encoder controllers, which are typically used on the embedded system (such as a Raspberry Pi Pico) to directly manage the motors and read rotational feedback.

______________________________________________________________________

### 1. `encoder_controller.h`: Quadrature Encoder Interface

The `EncoderController` reads the position and rotation of a DC motor using a quadrature encoder. It relies on GPIO interrupts for precise, real-time counting of the encoder ticks.

#### Class: `EncoderController`

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`EncoderController(...)`** | **Constructor.** Initializes pin configuration and encoder parameters. | `pinA`, `pinB`: GPIO pins for encoder channels A and B. <br> `pulsesPerRev`: Number of pulses per motor revolution. <br> `gearRatio`: Optional gear ratio between encoder shaft and wheel (default 1). | N/A |
| **`void begin()`** | **Initialization.** Configures GPIOs and attaches the static interrupt service routine (`encoderISR`) to the specified pins. | N/A | N/A |
| **`long getCount() const`** | **Raw Data.** Retrieves the current raw, thread-safe tick count of the encoder. | N/A | The total accumulated encoder ticks (long). |
| **`double getAngle() const`** | **Processed Data.** Calculates the current angle of rotation in degrees based on the tick count, `pulsesPerRev`, and `gearRatio`. | N/A | Current angle in degrees. |
| **`void reset()`** | **State Reset.** Sets the internal encoder tick count back to zero. | N/A | N/A |

______________________________________________________________________

### 2. `motor_controller.h`: DC Motor Driver (L298N)

The `MotorController` provides direct control over a single DC motor's direction and speed using a motor driver (like the L298N mini board) via Pulse Width Modulation (PWM) on the Raspberry Pi Pico.

#### Class: `MotorController`

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`MotorController(...)`** | **Constructor.** Initializes the GPIO pins connected to the L298N inputs. | `pinIn1`: GPIO pin connected to L298N IN1. <br> `pinIn2`: GPIO pin connected to L298N IN2. | N/A |
| **`void begin()`** | **Initialization.** Configures the specified GPIO pins for PWM output and sets up the necessary PWM slice/channel configuration. | N/A | N/A |
| **`void setPower(float power)`** | **Actuation.** Sets the motor speed and direction. A value of $100.0$ is maximum forward speed, $-100.0$ is maximum reverse speed, and $0.0$ is stopped. | `power`: Motor power percentage (range: $-100.0$ to $100.0$). | N/A |
| **`void stop(bool brake = false)`** | **Stopping.** Halts the motor. Can optionally apply dynamic braking by driving both direction pins high. | `brake`: If `true`, applies brake; otherwise, stops freely (coasting). | N/A |

______________________________________________________________________

### 3. `motor_speed_controller.h`: Closed-Loop Speed Regulator

The `MotorSpeedController` implements a closed-loop control system for motor speed. It combines the `MotorController` (actuator), the `EncoderController` (sensor feedback), and a `PIDController` (algorithm) to maintain a constant target speed in Revolutions Per Second (RPS).

#### Class: `MotorSpeedController`

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`MotorSpeedController(...)`** | **Constructor.** Links the motor and encoder instances and initializes the internal PID controller with gains. | `motor`: Reference to the `MotorController`. <br> `encoder`: Reference to the `EncoderController`. <br> `Kp`, `Ki`, `Kd`: PID gain constants (default Kp=120.0, Ki=0.0, Kd=0.0). | N/A |
| **`void setTargetRPS(double rps)`** | **Setpoint.** Sets the desired speed the controller should maintain. | `rps`: Target speed in Revolutions Per Second. | N/A |
| **`void update()`** | **Control Loop.** Calculates the current speed based on elapsed time and encoder changes, computes the required motor power using the PID algorithm, and applies the new power via the `MotorController`. | N/A | N/A |
| **`double getCurrentRPS() const`** | **Monitor.** Retrieves the motor speed calculated during the last `update()` cycle. | N/A | Current speed in RPS. |
| **`double getPower() const`** | **Monitor.** Retrieves the last motor power percentage (duty cycle) applied by the controller. | N/A | Last applied power ($-100.0$ to $100.0$). |


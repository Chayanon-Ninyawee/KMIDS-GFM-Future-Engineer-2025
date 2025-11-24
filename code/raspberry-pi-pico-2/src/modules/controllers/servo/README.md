## `servo_controller.h` Reference: Hobby RC Servo Driver

This header defines the `ServoController` class, which manages the control signal for standard hobby RC servos or any actuator that uses Pulse Width Modulation (PWM) for positioning. It leverages the precise hardware PWM capabilities of the Raspberry Pi Pico to generate the required pulse train (typically 50 Hz with 1000 µs to 2000 µs pulse width).

______________________________________________________________________

### Class: `ServoController`

The controller uses a linear mapping to translate a desired angle (e.g., $0^\\circ$ to $180^\\circ$) into a precise pulse width (in microseconds), which controls the servo's physical position.

#### Public Methods

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`ServoController(...)`** | **Constructor.** Sets up the servo's physical and electrical parameters. | `pin`: GPIO pin connected to the servo signal wire. <br> `maxAngle`: The maximum angle the servo can turn (default: $180.0^\\circ$). <br> `minPulseUs`: Pulse width (µs) corresponding to the minimum angle (default: 1000 µs). <br> `maxPulseUs`: Pulse width (µs) corresponding to `maxAngle` (default: 2000 µs). <br> `freqHz`: PWM frequency (default: 50 Hz, standard for hobby servos). | N/A |
| **`void begin()`** | **Initialization.** Configures the specified GPIO pin for PWM output, sets the required frequency (`freqHz`), and calculates the necessary internal PWM `wrap` value. | N/A | N/A |
| **`void setAngle(float angle)`** | **Angle Control.** Sets the servo to a desired angle. The input angle is linearly interpolated to the required pulse width (µs) defined by `minPulseUs` and `maxPulseUs`. | `angle`: Desired servo angle in degrees. | N/A |
| **`void setPulseWidth(uint16_t pulseUs)`** | **Direct Control.** Sets the PWM pulse width directly in microseconds, bypassing the angle calculation. Useful for fine-tuning or controlling non-servo devices. | `pulseUs`: The pulse width in microseconds ($\\mu s$) to apply. | N/A |

#### Private Members

| Member | Type | Description |
| :--- | :--- | :--- |
| **`pin_`, `slice_`, `channel_`** | Internal | GPIO pin and associated PWM hardware identifiers (slice and channel) on the Pico. |
| **`minPulseUs_`, `maxPulseUs_`** | Internal | The configured bounds for the pulse width in microseconds. |
| **`freqHz_`** | Internal | The configured PWM signal frequency in Hertz. |
| **`maxAngle_`** | Internal | The angle used as the upper bound for the `setAngle` mapping. |
| **`wrap_`** | Internal | The calculated maximum count value for the PWM hardware timer. |


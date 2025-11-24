## `pid_controller.h` Reference: Proportional-Integral-Derivative Control

This header defines a simple, yet robust, `PIDController` class used for feedback control systems. It computes a control output based on the proportional, integral, and derivative components of the error signal.

[Image of PID control loop diagram]

______________________________________________________________________

### Class: `PIDController`

| Term | Symbol | Description |
| :--- | :--- | :--- |
| **Proportional** | $P = K_p \\cdot e(t)$ | Scales the immediate error. |
| **Integral** | $I = K_i \\cdot \\int e(\\tau) d\\tau$ | Eliminates steady-state error (accumulates error over time). |
| **Derivative** | $D = K_d \\cdot \\frac{d e(t)}{d t}$ | Dampens oscillations (predicts future error based on rate of change). |

The final output is clamped between configurable minimum and maximum limits.

#### Public Methods

| Method | Description |
| :--- | :--- |
| **`PIDController(Kp, Ki, Kd, outputMin, outputMax)`** | **Constructor.** Initializes the controller with the required gains and output saturation limits. Default limits are $-100.0$ and $100.0$. |
| **`double update(double error, double dt)`** | **Core function.** Calculates the new PID output for the current time step. It applies output clamping and uses the time elapsed (`dt`) to correctly scale the integral and derivative terms. |
| **`void reset()`** | Clears the internal state of the controller. This sets the accumulated integral term (`integral_`) and the previous error (`lastError_`) back to zero. |
| **`void setActive(bool enable)`** | Enables or disables the PID computation. When disabled, `update()` should return a zero output. |
| **`bool isActive() const`** | Returns `true` if the controller is currently active. |
| **`void setGains(double Kp, double Ki, double Kd)`** | Sets new proportional, integral, and derivative gains. **Note:** This method also typically calls `reset()` to ensure the controller state aligns with the new tuning parameters. |

#### Private Members (Internal State)

| Member | Type | Description |
| :--- | :--- | :--- |
| **`Kp_`** | `double` | Proportional gain. |
| **`Ki_`** | `double` | Integral gain. |
| **`Kd_`** | `double` | Derivative gain. |
| **`integral_`** | `double` | The accumulated error term used by the integral component ($I$). |
| **`lastError_`** | `double` | The error value from the previous time step, used to calculate the derivative term ($D$). |
| **`outputMin_`** | `double` | The minimum allowed value for the calculated PID output (clamping limit). |
| **`outputMax_`** | `double` | The maximum allowed value for the calculated PID output (clamping limit). |
| **`active_`** | `bool` | Current activation state of the controller. |

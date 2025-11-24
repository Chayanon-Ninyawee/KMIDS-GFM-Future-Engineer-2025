## `pid_controller.h` Reference: Simple PID Control

This header defines the `PIDController` class, a standard implementation of a Proportional-Integral-Derivative (PID) controller. This utility is used for closed-loop control systems to accurately maintain an output value (e.g., steering angle, motor speed) based on the error between a desired setpoint and a measured value.

______________________________________________________________________

### Class: `PIDController`

The PID controller calculates a control output $u(t)$ based on three terms:

1. **Proportional (P):** Responds to the current error, $e(t)$.
1. **Integral (I):** Responds to the accumulation of past errors, removing steady-state errors.
1. **Derivative (D):** Responds to the rate of change of the error, dampening oscillations.

$\\text{Output} = K_p e(t) + K_i \\int e(t) dt + K_d \\frac{d e(t)}{d t}$

#### Public Methods

| Method | Description | Parameters | Returns |
| :--- | :--- | :--- | :--- |
| **`PIDController(...)`** | **Constructor.** Initializes the controller gains and sets the output limits. | `Kp`, `Ki`, `Kd`: The respective gain constants. <br> `outputMin`: Minimum allowable output value (default: -100.0). <br> `outputMax`: Maximum allowable output value (default: 100.0). | N/A |
| **`double update(double error, double dt)`** | **Core Calculation.** Computes the new PID output based on the current system error and the time elapsed since the last call. Includes integral term accumulation and output clamping. | `error`: The difference between the setpoint and the measured value. <br> `dt`: The time step $\\Delta t$ in seconds since the last call to `update()`. | The computed and clamped PID output value. |
| **`void reset()`** | **State Reset.** Clears the accumulated integral sum (`integral_`) and the previous error (`lastError_`) to reset the controller state. | N/A | N/A |

#### Private Members (Internal State)

| Member | Type | Description |
| :--- | :--- | :--- |
| **`Kp_`, `Ki_`, `Kd_`** | `double` | The proportional, integral, and derivative gain constants. |
| **`integral_`** | `double` | The accumulated sum of errors multiplied by $dt$, used for the integral term. |
| **`lastError_`** | `double` | The error value from the previous `update()` call, used to calculate the derivative term. |
| **`outputMin_`, `outputMax_`** | `double` | The bounds used to clamp the final output value. |

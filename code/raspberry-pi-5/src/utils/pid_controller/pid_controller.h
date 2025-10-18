#pragma once
#include <algorithm>

/**
 * @brief Simple PID controller class.
 *
 * Computes output based on proportional, integral, and derivative terms.
 */
class PIDController
{
public:
    /**
     * @brief Construct a new PIDController object.
     *
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param outputMin Minimum output limit
     * @param outputMax Maximum output limit
     */
    PIDController(double Kp, double Ki, double Kd, double outputMin = -100.0, double outputMax = 100.0);

    /**
     * @brief Update the PID output based on the current error and time step.
     *
     * @param error Current error (setpoint - measurement)
     * @param dt Time elapsed since last update (seconds)
     * @return double PID output
     */
    double update(double error, double dt);

    /**
     * @brief Reset the integral and derivative state.
     */
    void reset();

    /**
     * @brief Enable or disable the PID controller.
     * @param enable True to activate, false to deactivate.
     */
    void setActive(bool enable);

    /**
     * @brief Check if the controller is active.
     * @return True if active, false otherwise.
     */
    bool isActive() const {
        return active_;
    }

    /**
     * @brief Set new gains for the PID controller.
     *
     * This will also reset the controller's internal state.
     * @param Kp New proportional gain.
     * @param Ki New integral gain.
     * @param Kd New derivative gain.
     */
    void setGains(double Kp, double Ki, double Kd);

private:
    double Kp_;        /**< Proportional gain */
    double Ki_;        /**< Integral gain */
    double Kd_;        /**< Derivative gain */
    double integral_;  /**< Accumulated integral term (for I component) */
    double lastError_; /**< Previous error (for D component calculation) */
    double outputMin_; /**< Minimum allowed output (clamping) */
    double outputMax_; /**< Maximum allowed output (clamping) */
    bool active_;
};

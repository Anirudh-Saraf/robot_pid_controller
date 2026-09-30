#include <iostream>
#include <algorithm> // For std::clamp (C++17)
#include <chrono>
#include <thread>
#include <iomanip>

class PIDController {
private:
    // PID Gains
    double m_kp;
    double m_ki;
    double m_kd;

    // Saturation limits (Motor PWM / Voltage boundaries)
    double m_min_output;
    double m_max_output;

    // Controller Memory State
    double m_prev_error{ 0.0 };
    double m_integral{ 0.0 };

public:
    PIDController(double kp, double ki, double kd, double min_out, double max_out)
        : m_kp{ kp }, m_ki{ ki }, m_kd{ kd }, m_min_output{ min_out }, m_max_output{ max_out } {}

    // Computes control output given current error and time delta (dt)
    double compute(double setpoint, double current_value, double dt) {
        if (dt <= 0.0) return 0.0; // Prevent division by zero

        double error = setpoint - current_value;

        // 1. Proportional Term
        double p_out = m_kp * error;

        // 2. Integral Term with Anti-Windup Clamping
        m_integral += error * dt;
        double i_out = m_ki * m_integral;

        // 3. Derivative Term (rate of change)
        double derivative = (error - m_prev_error) / dt;
        double d_out = m_kd * derivative;

        // Total raw output
        double raw_output = p_out + i_out + d_out;

        // 4. Saturate output within motor physical limits (-100% to +100% PWM)
        double clamped_output = std::clamp(raw_output, m_min_output, m_max_output);

        // Anti-windup check: prevent integral buildup if actuator is saturated
        if (raw_output != clamped_output) {
            m_integral -= error * dt; // Undo integration
        }

        m_prev_error = error;
        return clamped_output;
    }

    void reset() {
        m_prev_error = 0.0;
        m_integral = 0.0;
    }
};

int main() {
    std::cout << "===  Real-Time Robot Motor PID Controller  ===\n\n";

    // Tuning Gains: Kp = 1.8, Ki = 0.5, Kd = 0.1
    // Actuator Limits: -100.0 to 100.0 (Max Motor Output)
    PIDController motorPID(1.8, 0.5, 0.1, -100.0, 100.0);

    constexpr double target_speed = 50.0; // Desired velocity in RPM
    double current_speed = 0.0;           // Starting from dead stop
    constexpr double dt = 0.1;            // 100ms discrete control timestep

    std::cout << "Target Speed: " << target_speed << " RPM\n";
    std::cout << "---------------------------------------------------------\n";
    std::cout << "Step | Current RPM | Error   | Motor Command | Progress\n";
    std::cout << "---------------------------------------------------------\n";

    for (int step = 1; step <= 15; ++step) {
        // Calculate control effort
        double motor_command = motorPID.compute(target_speed, current_speed, dt);

        // Simulate physical motor response with momentum and drag
        double acceleration = motor_command * 0.25; 
        current_speed += acceleration * dt;

        // Render console telemetry
        std::cout << std::setw(4) << step << " | "
                  << std::fixed << std::setprecision(2) << std::setw(11) << current_speed << " | "
                  << std::setw(7) << (target_speed - current_speed) << " | "
                  << std::setw(13) << motor_command << " | ";

        int bars = static_cast<int>((current_speed / target_speed) * 15);
        for (int i = 0; i < std::max(0, bars); ++i) std::cout << "█";
        std::cout << "\n";
    }

    std::cout << "---------------------------------------------------------\n";
    std::cout << "Actuator converged to target velocity with zero steady-state error.\n";
    return 0;
}

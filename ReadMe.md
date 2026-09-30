# Real-Time Discrete PID Controller with Anti-Windup

![Language](https://img.shields.io/badge/Language-C%2B%2B17%20%2F%20C%2B%2B20-blue)
![Platform](https://img.shields.io/badge/Platform-Ubuntu%20Linux-orange)
![Control Theory](https://img.shields.io/badge/Control%20Loop-Closed--Loop%20Feedback-brightgreen)

A modular, production-ready implementation of a discrete Proportional-Integral-Derivative (PID) controller designed for robotic actuators (brushed/brushless motors, servo joints). This system includes integral anti-windup clamping to prevent overshoot and physical damage during actuator saturation.

---

## 🛰️ How It Works

A PID controller measures the difference between a desired target velocity or position (setpoint) and the current physical state, applying three corrective terms:

$$\text{Output}(t) = K_p \cdot e(t) + K_i \int e(t)\,dt + K_d \frac{de(t)}{dt}$$

*   **Proportional ($K_p$):** Reacts immediately to the current error.
*   **Integral ($K_i$):** Accumulates historical error over time to eliminate steady-state offsets caused by friction or gravity.
*   **Derivative ($K_d$):** Predicts future error by calculating the current rate of change, acting as a dampener to prevent system oscillation.

### 🛡️ The Anti-Windup Guard

If a robot motor is blocked physically or commanded to spin beyond its maximum voltage, the error remains high, causing the integral term to grow infinitely (windup). When the blockage is cleared, the accumulated integral causes a massive, dangerous overshoot.

This implementation utilizes **Conditional Integration Clamping**:
1. Calculate the raw output.
2. Clamp the output to physical hardware limits (`std::clamp`).
3. If the controller is saturated (output is restricted by limits), further accumulation of the integral term is frozen.

---

## 🛠️ Compilation & Execution

To compile and run the motor simulation under Ubuntu, execute the following commands in your terminal:

```bash
# Compile using C++17 standard
g++ -std=c++17 pid_controller.cpp -o pid_sim

# Run the simulation binary
./pid_sim

#pragma once
#include <cmath>
#include <array>
#include <algorithm>
#include <limits>

class FourWheelSteeringSolver {
public:
    struct Cmd {
        double vx; // m/s
        double vy; // m/s
        double wz; // rad/s
    };

    struct WheelState {
        double speed; // m/s
        double angle; // rad (steering)
    };

    // Configuration parameters
    struct Config {
        double track_width;       // Distance between L/R wheels
        double wheel_base;        // Distance between F/R wheels
        double error_alpha;       // Filter factor [0.0 - 1.0]. Lower = More "Memory" (sluggish but smooth)
        double gain_k;            // Sharpness of EMA weighting vs steering error. Higher = more responsive.
        double max_steer_angle;   // Physical limit of the servo (rad)
        double cmd_deadzone_lin;  // Command deadzone for linear speed (m/s)
        double cmd_deadzone_ang;  // Command deadzone for angular speed (rad/s)
        double vel_eps;           // Wheel speed epsilon for undefined direction (m/s)
    };

    FourWheelSteeringSolver(Config cfg)
        : cfg_(cfg), filtered_steer_{0.0, 0.0, 0.0, 0.0}, steer_initialized_(false) {}

    /**
     * @brief The Core Update Loop
     * @param cmd The desired body twist (filtered cmd_vel)
     * @param current_steering The actual measured angles of the 4 wheels [FL, FR, RL, RR]
     * @return The target commands for the motors [FL, FR, RL, RR]
     */
    std::array<WheelState, 4> solve(const Cmd& cmd, 
                                    const std::array<double, 4>& current_steering) {
        
        std::array<WheelState, 4> targets;

        // 1. Calculate Geometry (Lever Arms)
        // FL (+x, +y), FR (+x, -y), RL (-x, +y), RR (-x, -y)
        const double x_offset = cfg_.wheel_base / 2.0;
        const double y_offset = cfg_.track_width / 2.0;
        const double x_signs[4] = {1, 1, -1, -1};
        const double y_signs[4] = {1, -1, 1, -1};

        // Check if command is effectively zero (Deadzone)
        const bool is_command_zero =
            (std::hypot(cmd.vx, cmd.vy) < cfg_.cmd_deadzone_lin) &&
            (std::abs(cmd.wz) < cfg_.cmd_deadzone_ang);

        if (!steer_initialized_) {
            filtered_steer_ = current_steering;
            steer_initialized_ = true;
        }

        for (int i = 0; i < 4; ++i) {
            // --- STEP A: Pure Inverse Kinematics ---
            // Velocity of the wheel contact point = V_body + W_body x R_wheel
            double vx_wheel = cmd.vx - cmd.wz * (y_offset * y_signs[i]);
            double vy_wheel = cmd.vy + cmd.wz * (x_offset * x_signs[i]);

            double raw_speed = std::hypot(vx_wheel, vy_wheel);
            double raw_angle = std::atan2(vy_wheel, vx_wheel);

            // --- STEP B: Singularity Handling (The "Stop" Logic) ---
            if (is_command_zero) {
                // If robot is commanded to stop, DO NOT snap wheels to 0.0.
                // Lock them to their current angle. This prevents "jitter" at rest.
                targets[i].angle = current_steering[i];
                targets[i].speed = 0.0;
                filtered_steer_[i] = current_steering[i];
                continue;
            }

            // --- STEP C: Undefined atan2 handling ---
            // If the wheel velocity is effectively zero, keep prior steering.
            const bool undefined_dir = (raw_speed < cfg_.vel_eps);
            if (undefined_dir) {
                raw_angle = filtered_steer_[i];
            }

            // --- STEP D: Optimization (The "Flip" Logic) ---
            // We want to avoid turning the wheel 180 degrees if we can just reverse the motor.
            if (!undefined_dir) {
                double diff = normalizeAngle(raw_angle - current_steering[i]);
                if (std::abs(diff) > M_PI_2) { // > 90 degrees
                    raw_angle = normalizeAngle(raw_angle + M_PI);
                    raw_speed *= -1.0;
                }
            }

            // --- STEP E: Per-wheel weighted EMA on steering command ---
            const double steer_err =
                std::abs(normalizeAngle(raw_angle - current_steering[i]));
            const double weight =
                (cfg_.gain_k > 0.0) ? (1.0 - std::exp(-cfg_.gain_k * steer_err)) : 1.0;
            const double alpha = std::clamp(cfg_.error_alpha * weight, 0.0, 1.0);
            const double delta = normalizeAngle(raw_angle - filtered_steer_[i]);
            filtered_steer_[i] = normalizeAngle(filtered_steer_[i] + alpha * delta);

            targets[i].angle = filtered_steer_[i];
            targets[i].speed = raw_speed;
        }

        return targets;
    }

private:
    Config cfg_;
    std::array<double, 4> filtered_steer_;
    bool steer_initialized_;

    // Helper: Normalize angle to [-pi, pi]
    double normalizeAngle(double angle) {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }
};

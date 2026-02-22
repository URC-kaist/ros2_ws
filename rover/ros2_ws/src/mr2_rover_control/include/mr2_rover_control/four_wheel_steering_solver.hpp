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
        double max_steer_angle;   // Steering clamp for solver output (rad)
        double steer_speed_no_atten;   // No speed attenuation below this error (rad)
        double steer_speed_full_atten; // Full stop at/above this error (rad)
        double cmd_deadzone_lin;  // Command deadzone for linear speed (m/s)
        double cmd_deadzone_ang;  // Command deadzone for angular speed (rad/s)
        double vel_eps;           // Wheel speed epsilon for undefined direction (m/s)
    };

    FourWheelSteeringSolver(Config cfg)
        : cfg_(cfg), filtered_steer_{0.0, 0.0, 0.0, 0.0}, steer_initialized_(false) {}

    void setErrorAlpha(double error_alpha) { cfg_.error_alpha = std::clamp(error_alpha, 0.0, 1.0); }

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

            // Clamp steering to configured limits (e.g., +/- 90 deg).
            raw_angle = clampSteer(raw_angle);

            // --- STEP E: Per-wheel weighted EMA on steering command ---
            const double steer_err =
                std::abs(normalizeAngle(raw_angle - current_steering[i]));
            const double weight =
                (cfg_.gain_k > 0.0) ? (1.0 - std::exp(-cfg_.gain_k * steer_err)) : 1.0;
            const double alpha = (cfg_.error_alpha >= 1.0)
                                     ? 1.0
                                     : std::clamp(cfg_.error_alpha * weight, 0.0, 1.0);
            const double delta = normalizeAngle(raw_angle - filtered_steer_[i]);
            filtered_steer_[i] = normalizeAngle(filtered_steer_[i] + alpha * delta);
            filtered_steer_[i] = clampSteer(filtered_steer_[i]);

            targets[i].angle = filtered_steer_[i];
            targets[i].speed = raw_speed;
        }

        // Global speed attenuation based on the worst steering alignment.
        double max_align_err = 0.0;
        for (int i = 0; i < 4; ++i) {
            const double align_err =
                std::abs(normalizeAngle(targets[i].angle - current_steering[i]));
            if (align_err > max_align_err) {
                max_align_err = align_err;
            }
        }
        const double speed_scale = steerSpeedScale(max_align_err);
        for (int i = 0; i < 4; ++i) {
            targets[i].speed *= speed_scale;
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

    double clampSteer(double angle) {
        const double limit = cfg_.max_steer_angle;
        if (limit <= 0.0) {
            return normalizeAngle(angle);
        }
        return std::clamp(normalizeAngle(angle), -limit, limit);
    }

    double steerSpeedScale(double steer_err) {
        const double start = std::max(0.0, cfg_.steer_speed_no_atten);
        const double end = std::max(start, cfg_.steer_speed_full_atten);
        if (end <= start + 1e-6) {
            return (steer_err <= start) ? 1.0 : 0.0;
        }
        if (steer_err <= start) {
            return 1.0;
        }
        if (steer_err >= end) {
            return 0.0;
        }
        const double t = (steer_err - start) / (end - start);
        return std::clamp(1.0 - t, 0.0, 1.0);
    }
};

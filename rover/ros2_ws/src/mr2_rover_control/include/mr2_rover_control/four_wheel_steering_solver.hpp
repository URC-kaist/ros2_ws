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
        double gain_k;            // Sharpness of the velocity drop-off. Higher = Stop harder on error.
        double max_steer_angle;   // Physical limit of the servo (rad)
    };

    FourWheelSteeringSolver(Config cfg) 
        : cfg_(cfg), filtered_system_error_(0.0) {}

    /**
     * @brief The Core Update Loop
     * @param cmd The desired body twist (filtered cmd_vel)
     * @param current_steering The actual measured angles of the 4 wheels [FL, FR, RL, RR]
     * @return The target commands for the motors [FL, FR, RL, RR]
     */
    std::array<WheelState, 4> solve(const Cmd& cmd, 
                                    const std::array<double, 4>& current_steering) {
        
        std::array<WheelState, 4> targets;
        double max_instant_error = 0.0;

        // 1. Calculate Geometry (Lever Arms)
        // FL (+x, +y), FR (+x, -y), RL (-x, +y), RR (-x, -y)
        const double x_offset = cfg_.wheel_base / 2.0;
        const double y_offset = cfg_.track_width / 2.0;
        const double x_signs[4] = {1, 1, -1, -1};
        const double y_signs[4] = {1, -1, 1, -1};

        // Check if command is effectively zero (Deadzone)
        bool is_command_zero = (std::hypot(cmd.vx, cmd.vy) < 1e-3) && (std::abs(cmd.wz) < 1e-3);

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
            } else {
                // --- STEP C: Optimization (The "Flip" Logic) ---
                // We want to avoid turning the wheel 180 degrees if we can just reverse the motor.
                
                // 1. Normalize angle difference to range [-pi, pi]
                double diff = normalizeAngle(raw_angle - current_steering[i]);

                // 2. Check if the "Reverse" solution is closer
                // If the target is > 90 degrees away, it's faster to flip the wheel 180 deg
                // and reverse the speed.
                if (std::abs(diff) > M_PI_2) { // > 90 degrees
                    raw_angle = normalizeAngle(raw_angle + M_PI);
                    raw_speed *= -1.0;
                }

                targets[i].angle = raw_angle;
                targets[i].speed = raw_speed;
            }

            // --- STEP D: Track the "Suffering" ---
            // How far is the wheel from where it needs to be?
            // This time, we calculate the error of the *optimized* angle.
            double current_diff = std::abs(normalizeAngle(targets[i].angle - current_steering[i]));
            if (current_diff > max_instant_error) {
                max_instant_error = current_diff;
            }
        }

        // --- STEP E: The Memory Filter (Control System Logic) ---
        // Instead of reacting to the instant error (which causes jitter), 
        // we feed the error into a Low Pass Filter (Exponential Moving Average).
        // If the robot makes a sudden move, this error spikes and decays slowly.
        filtered_system_error_ = (1.0 - cfg_.error_alpha) * filtered_system_error_ 
                               + cfg_.error_alpha * max_instant_error;

        // --- STEP F: The Drive Gain (Gating) ---
        // Calculate a scalar [0.0 to 1.0] based on the filtered error.
        // Formula: Gain = e^(-k * error)
        // If error is 0, Gain = 1.0. If error is high, Gain approaches 0.0.
        double drive_gain = std::exp(-cfg_.gain_k * filtered_system_error_);

        // Apply gain ONLY to speed. Steering servos always get full authority.
        for (int i = 0; i < 4; ++i) {
            targets[i].speed *= drive_gain;
            
            // Optional: Hard clamp for safety (prevent driving while steering is totally wrong)
            if (filtered_system_error_ > (45.0 * M_PI / 180.0)) {
                targets[i].speed = 0.0;
            }
        }

        return targets;
    }

private:
    Config cfg_;
    double filtered_system_error_;

    // Helper: Normalize angle to [-pi, pi]
    double normalizeAngle(double angle) {
        angle = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (angle < 0) angle += 2.0 * M_PI;
        return angle - M_PI;
    }
};
#pragma once

class PidController {
public:
    PidController(float kp, float ki, float kd, float setpoint, float sample_time = 0.1)
        : kp_(kp), ki_(ki), kd_(kd), setpoint_(setpoint), sample_time_(sample_time),
          integral_(0.0), previous_error_(0.0), previous_time_(std::chrono::high_resolution_clock::now()) {}

    float compute(float current_value) {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> elapsed = current_time - previous_time_;
        float dt = elapsed.count();

        if (dt >= sample_time_) {
            float error = setpoint_ - current_value;
            integral_ += error * dt;
            float derivative = (error - previous_error_) / dt;

            float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

            previous_error_ = error;
            previous_time_ = current_time;

            return output;
        }

        return 0.0;
    }

    void setTunings(float kp, float ki, float kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void setSetpoint(float setpoint) {
        setpoint_ = setpoint;
    }

    void reset() {
        integral_ = 0.0;
        previous_error_ = 0.0;
    }

private:
    float kp_;
    float ki_;
    float kd_;
    float setpoint_;
    float sample_time_;

    float integral_;
    float previous_error_;

    std::chrono::high_resolution_clock::time_point previous_time_;
};

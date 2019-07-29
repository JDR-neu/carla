#ifndef LOGNITUDINAL_PID_CONTROLLER_H
#define LOGNITUDINAL_PID_CONTROLLER_H

#include <vector>
#include <numeric>
#include <iostream>


class LongitudinalPIDController {
    public:
    float kp = 1.0f, kd = 0.0f, ki = 0.0f, dt = 0.03f;
    float RunStep(float target_speed, float current_speed) {
        float err = target_speed - current_speed;
        error_buf.emplace_back(err);
        float de = 0.0f, ie = 0.0f;
        if(error_buf.size() >= 2) {
            de = (error_buf.back() - error_buf[error_buf.size() - 2]) / dt;
            ie = std::accumulate(error_buf.begin() , error_buf.end() , 0.0f) * dt;
        } else {
            de = 0.0f;
            ie = 0.0f;
        }
        float throttle = kp * err + kd * de / dt + ki * ie * dt;
        std::cout << "in pid controller, throttle 1 = " << throttle << std::endl;
        throttle = std::min(1.0f, std::max(0.0f, throttle));
        std::cout << "in pid controller, throttle 2 = " << throttle << std::endl;
        return throttle;
    }
    private:
    std::vector<float> error_buf;
};

#endif //LOGNITUDINAL_PID_CONTROLLER_H
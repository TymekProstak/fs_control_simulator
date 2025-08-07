#include "steer_geometry.h"

namespace metzler_model {

    SteerGeometry::SteerGeometry(const steer_geometry_params& params, const steer_geometry_state& initial_state = steer_geometry_state{0.0, 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0})
        : params_(params), state_(initial_state) {}

        


    slip_angles SteerGeometry::calculate_slip_angles(double vx, double vy, double yaw_rate) const {
    
    // Calculate the slip angles based on the vehicle's velocity and yaw rate.
    
    slip_angles result;

    double lf = params_.lf;
    double lr = params_.lr;
    double wf = params_.wheel_distance_front;
    double wr = params_.wheel_distance_rear;

    // Zakładam, że masz w state_ kąty skrętu kół
    result.rear_left_slip_angle = std::atan2(vy - lr * yaw_rate, vx - yaw_rate * wr / 2.0);
    result.rear_right_slip_angle = std::atan2(vy - lr * yaw_rate, vx + yaw_rate * wr / 2.0);

    result.front_left_slip_angle = std::atan2(vy + lf * yaw_rate, vx - yaw_rate * wf / 2.0) - state_.front_left_steer_angle;
    result.front_right_slip_angle = std::atan2(vy + lf * yaw_rate, vx + yaw_rate * wf / 2.0) - state_.front_right_steer_angle;

    // Ogranicz do zakresu [-pi, pi]
    result.rear_left_slip_angle = std::fmod(result.rear_left_slip_angle + M_PI, 2 * M_PI) - M_PI;
    result.rear_right_slip_angle = std::fmod(result.rear_right_slip_angle + M_PI, 2 * M_PI) - M_PI;
    result.front_left_slip_angle = std::fmod(result.front_left_slip_angle + M_PI, 2 * M_PI) - M_PI;
    result.front_right_slip_angle = std::fmod(result.front_right_slip_angle + M_PI, 2 * M_PI) - M_PI;

    return result;
}

     steer_angles SteerGeometry::calculate_steer_angles() const  {

        
    



    }


    void SteerGeometry::calculate_and_set_steer_angles() {
        // Calculate the steer angles based on the vehicle's geometry and the actual steering on the steering column.
        // For now, assuming an ideal Ackerman steering geometry.


        steer_angles new_steer_angles = calculate_steer_angles();
        set_steer_angles(new_steer_angles);


    }
    void SteerGeometry::calculate_and_set_slip_angles(double vx, double vy, double yaw_rate) {
        // Calculate the slip angles based on the vehicle's velocity and yaw rate.

        slip_angles new_slip_angles = calculate_slip_angles(vx, vy, yaw_rate);
        set_slip_angles(new_slip_angles);
    }




    





}
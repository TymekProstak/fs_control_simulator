#include "steer_geometry.h"

namespace metzler_model {

    SteerGeometry::SteerGeometry(const steer_geometry_params& params, const steer_geometry_state& initial_state )
        : params_(params), state_(initial_state) {}

        


     steer_angles SteerGeometry::calculate_steer_angles() const  {


        // Calculate the steer angles based on the vehicle's geometry and the actual steering on the steering column.
        // For now, assuming an ideal Ackerman steering geometry.

        steer_angles result;

        // function calculates the steer angles based on the vehicle's geometry and the actual steering on the steering column.
        // angle on steering column remains the same 


        result.steer_angle_on_column = state_.steer_angle_on_column;

        double turning radius = params_.wheelbase / std::tan(state_.steer_angle_on_column);

        result.front_left_steer_angle = std::atan2( params_.wheelbase, turning radius - params_.wheel_distance_front / 2.0);
        result.front_right_steer_angle = std::atan2( params_.wheelbase, turning radius + params_.wheel_distance_front / 2.0);

        // Ensure angles are within the range of [-pi, pi]
        result.front_left_steer_angle = std::fmod(result.front_left_steer_angle + M_PI, 2 * M_PI) - M_PI;
        result.front_right_steer_angle = std::fmod(result.front_right_steer_angle + M_PI, 2 * M_PI) - M_PI;

        return result;


    


    }


    void SteerGeometry::calculate_and_set_steer_angles() {

        // Calculate the steer angles based on the vehicle's geometry and the actual steering on the steering column.
        // For now, assuming an ideal Ackerman steering geometry.


        steer_angles new_steer_angles = calculate_steer_angles();
        set_steer_angles(new_steer_angles);


    }



    std::vector<double> SteerGeometry::state_to_vector() const {
            return {state_.front_left_steer_angle, state_.front_right_steer_angle, state_.steer_angle_on_column};
        }

    void SteerGeometry::vector_to_state(const std::vector<double>& vec) {
        if (vec.size() != 3) {
            throw std::invalid_argument("Vector size must be 3");
        }
        state_.front_left_steer_angle = vec[0];
        state_.front_right_steer_angle = vec[1];
        state_.steer_angle_on_column = vec[2];
        
    }

    double Engine::derivative_steer(double steer_input) const {
            
            // 1 rzędowy układ kierowniczy
            double derivative = (steer_input - state_.steer_actual) / params_.T;

            // TODO : Jakieś filtry górno/dolno przepustowe ? 

            return derivative;
    }

    void SteerGeometry::update(double steer_input, double dt) {
            // Calculate the derivative of the steer angle
            double derivative = derivative_steer(steer_input);  
            // Update the state using Euler's method
            state_.steer_angle_on_column += derivative * dt;
    }


        


} // namespace metzler_model
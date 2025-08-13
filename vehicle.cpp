#include "vehicle.h"

namespace metzler_model {



    Vehicle::Vehicle(ParamsInitializer& params_initializer)
            : vehicle_body(params_initializer.get_vehicle_body_params()),
              steer_geometry(params_initializer.get_steer_geometry_params()),
              tire_force_model_FL(params_initializer.get_tire_force_model_params()),
              tire_force_model_FR(params_initializer.get_tire_force_model_params()),
              tire_force_model_RL(params_initializer.get_tire_force_model_params()),
              tire_force_model_RR(params_initializer.get_tire_force_model_params()),
              engine_FL(params_initializer.get_engine_params()),
              engine_FR(params_initializer.get_engine_params()),
              engine_RL(params_initializer.get_engine_params()),
              engine_RR(params_initializer.get_engine_params()),
              tire_FL(params_initializer.get_tire_params()),
              tire_FR(params_initializer.get_tire_params()),
              tire_RL(params_initializer.get_tire_params()),
              tire_RR(params_initializer.get_tire_params()),
              normal_forces(params_initializer.get_normal_model_forces_params())
        {}

    void Vehicle::prepare_system(){

        ///// ****** prepering a nedded parameters of functions  **** ////////////////////////////////////////////


        /// Vehicle Body Kinematics ////////////////////////////////////////////////////////////////////

        double vx = vehicle_body.get_velocity_x();
        double vy = vehicle_body.get_velocity_y();
        double yaw_rate = vehicle_body.get_yaw_rate();
        double ax = vehicle_body.get_acceleration_x();
        double ay = vehicle_body.get_acceleration_y();

        /////////////////////////////////////////////////////////////////////////////////////////////////

        ///  Steering Angles ////////////////////////////////////////////////////////////////////////////


        steer_angles steer_angles = steer_geometry.calculate_set_and_return_steer_angles();
        double front_left_steer_angle = steer_angles.front_left_steer_angle;
        double front_right_steer_angle = steer_angles.front_right_steer_angle;

        /////////////////////////////////////////////////////////////////////////////////////////////////

        //** Normal Forces - depending on kinematics of vehicle body due to load transfer and areo forces **///////////////

        normal_model_forces_output = normal_forces.calculate_set_and_return_normal_forces( vx, vy, ax, ay);

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //// Now prepering tires - > values of steer angels need to be puted, tires then return value of slips ////////////////

        tire_FL.set_steering_angle(front_left_steer_angle);
        tire_FR.set_steering_angle(front_right_steer_angle);
        tire_RL.set_steering_angle(0.0); // rear tires are not steerable
        tire_RR.set_steering_angle(0.0); // rear tires are not steerable

        // Slips

        double front_left_slip_ratio = tire_FL.calculate_set_and_return_slip_ratio(vx, vy, yaw_rate);
        double front_left_slip_angle = tire_FL.calculate_set_and_return_slip_angle(vx, vy, yaw_rate);
        double front_right_slip_ratio = tire_FR.calculate_set_and_return_slip_ratio(vx, vy, yaw_rate);
        double front_right_slip_angle = tire_FR.calculate_set_and_return_slip_angle(vx, vy, yaw_rate);
        double rear_left_slip_ratio = tire_RL.calculate_set_and_return_slip_ratio(vx, vy, yaw_rate);
        double rear_left_slip_angle = tire_RL.calculate_set_and_return_slip_angle(vx, vy, yaw_rate);
        double rear_right_slip_ratio = tire_RR.calculate_set_and_return_slip_ratio(vx, vy, yaw_rate);
        double rear_right_slip_angle = tire_RR.calculate_set_and_return_slip_angle(vx, vy, yaw_rate);

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        

    }


    void Vehicle::update_vehicle_state(vehicle_input input,double dt) {




    }

    void Vehicle::RK4_integration_step(vehicle_input current_input, double dt) {
    // Implement the RK4 integration step here
    









    }

} // namespace metzler_model


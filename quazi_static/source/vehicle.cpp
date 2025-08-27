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

    Vehicle::Vehicle(const Vehicle& other)
    : current_state(other.current_state),
      current_input(other.current_input),
      current_slips_and_forces(other.current_slips_and_forces),
      vehicle_body(other.vehicle_body),
      steer_geometry(other.steer_geometry),
      tire_force_model_FL(other.tire_force_model_FL),
      tire_force_model_FR(other.tire_force_model_FR),
      tire_force_model_RL(other.tire_force_model_RL),
      tire_force_model_RR(other.tire_force_model_RR),
      engine_FL(other.engine_FL),
      engine_FR(other.engine_FR),
      engine_RL(other.engine_RL),
      engine_RR(other.engine_RR),
      tire_FL(other.tire_FL),
      tire_FR(other.tire_FR),
      tire_RL(other.tire_RL),
      tire_RR(other.tire_RR),
      normal_forces(other.normal_forces),
      forces_sumed(other.forces_sumed)
    {}


    void Vehicle::add_other_state_to_this(const vehicle_state& other) {
      current_state.x += other.x;
      current_state.y += other.y;
      current_state.yaw += other.yaw;
      current_state.vx += other.vx;
      current_state.vy += other.vy;
      current_state.yaw_rate += other.yaw_rate;
      current_state.ax += other.ax;
      current_state.ay += other.ay;
      current_state.torque_FL += other.torque_FL;
      current_state.torque_FR += other.torque_FR;
      current_state.torque_RL += other.torque_RL;
      current_state.torque_RR += other.torque_RR;
      current_state.steer_angle_on_column += other.steer_angle_on_column;
      current_state.omega_FL += other.omega_FL;
      current_state.omega_FR += other.omega_FR;
      current_state.omega_RL += other.omega_RL;
      current_state.omega_RR += other.omega_RR;
    }
    void Vehicle::add_this_state_to_other(vehicle_state& other) const {

      other.x += current_state.x;
      other.y += current_state.y;
      other.yaw += current_state.yaw;
      other.vx += current_state.vx;
      other.vy += current_state.vy;
      other.yaw_rate += current_state.yaw_rate;
      other.ax += current_state.ax;
      other.ay += current_state.ay;
      other.torque_FL += current_state.torque_FL;
      other.torque_FR += current_state.torque_FR;
      other.torque_RL += current_state.torque_RL;
      other.torque_RR += current_state.torque_RR;
      other.steer_angle_on_column += current_state.steer_angle_on_column;
      other.omega_FL += current_state.omega_FL;
      other.omega_FR += current_state.omega_FR;
      other.omega_RL += current_state.omega_RL;
      other.omega_RR += current_state.omega_RR;
      

    }

    void Vehicle::subtract_other_state_from_this(const vehicle_state& other) {
      current_state.x -= other.x;
      current_state.y -= other.y;
      current_state.yaw -= other.yaw;
      current_state.vx -= other.vx;
      current_state.vy -= other.vy;
      current_state.yaw_rate -= other.yaw_rate;
      current_state.ax -= other.ax;
      current_state.ay -= other.ay;
      current_state.torque_FL -= other.torque_FL;
      current_state.torque_FR -= other.torque_FR;
      current_state.torque_RL -= other.torque_RL;
      current_state.torque_RR -= other.torque_RR;
      current_state.steer_angle_on_column -= other.steer_angle_on_column;
      current_state.omega_FL -= other.omega_FL;
      current_state.omega_FR -= other.omega_FR;
      current_state.omega_RL -= other.omega_RL;
      current_state.omega_RR -= other.omega_RR;
    }
    void Vehicle::subtract_this_state_from_other(vehicle_state& other) const {
      other.x -= current_state.x;
      other.y -= current_state.y;
      other.yaw -= current_state.yaw;
      other.vx -= current_state.vx;
      other.vy -= current_state.vy;
      other.yaw_rate -= current_state.yaw_rate;
      other.ax -= current_state.ax;
      other.ay -= current_state.ay;
      other.torque_FL -= current_state.torque_FL;
      other.torque_FR -= current_state.torque_FR;
      other.torque_RL -= current_state.torque_RL;
      other.torque_RR -= current_state.torque_RR;
      other.steer_angle_on_column -= current_state.steer_angle_on_column;
      other.omega_FL -= current_state.omega_FL;
      other.omega_FR -= current_state.omega_FR;
      other.omega_RL -= current_state.omega_RL;
      other.omega_RR -= current_state.omega_RR;
    }
    void Vehicle::multiply_state_with_scalar(double scalar) {
      current_state.x *= scalar;
      current_state.y *= scalar;
      current_state.yaw *= scalar;
      current_state.vx *= scalar;
      current_state.vy *= scalar;
      current_state.yaw_rate *= scalar;
      current_state.ax *= scalar;
      current_state.ay *= scalar;
      current_state.torque_FL *= scalar;
      current_state.torque_FR *= scalar;
      current_state.torque_RL *= scalar;
      current_state.torque_RR *= scalar;
      current_state.steer_angle_on_column *= scalar;
      current_state.omega_FL *= scalar;
      current_state.omega_FR *= scalar;
      current_state.omega_RL *= scalar;
      current_state.omega_RR *= scalar;
    }

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
        double front_left_normal_force = normal_model_forces_output.front_left_force;
        double front_right_normal_force = normal_model_forces_output.front_right_force;
        double rear_left_normal_force = normal_model_forces_output.rear_left_force;
        double rear_right_normal_force = normal_model_forces_output.rear_right_force;

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

        /// Now calculating lateral and longitudal forces from Tires using friction model ///////////////////////////////

        tire_force_model_output_in_vehicle_frame output_FL = tire_force_model_FL.calculate_set_and_return_tire_force_in_vehicle_frame(front_left_slip_ratio, front_left_slip_angle, front_left_normal_force, front_left_steer_angle);
        tire_force_model_output_in_vehicle_frame output_FR = tire_force_model_FR.calculate_set_and_return_tire_force_in_vehicle_frame(front_right_slip_ratio, front_right_slip_angle, front_right_normal_force, front_right_steer_angle);
        tire_force_model_output_in_vehicle_frame output_RL = tire_force_model_RL.calculate_set_and_return_tire_force_in_vehicle_frame(rear_left_slip_ratio, rear_left_slip_angle, rear_left_normal_force, 0.0);
        tire_force_model_output_in_vehicle_frame output_RR = tire_force_model_RR.calculate_set_and_return_tire_force_in_vehicle_frame(rear_right_slip_ratio, rear_right_slip_angle, rear_right_normal_force, 0.0);


        tire_force_model_output output_FL_tire_frame = tire_force_model_FL.calculate_set_and_return_tire_force(front_left_slip_ratio, front_left_slip_angle, front_left_normal_force, front_left_steer_angle); 
        tire_force_model_output output_FR_tire_frame = tire_force_model_FR.calculate_set_and_return_tire_force(front_right_slip_ratio, front_right_slip_angle, front_right_normal_force, front_right_steer_angle);
        tire_force_model_output output_RL_tire_frame = tire_force_model_RL.calculate_set_and_return_tire_force(rear_left_slip_ratio, rear_left_slip_angle, rear_left_normal_force, 0.0);
        tire_force_model_output output_RR_tire_frame = tire_force_model_RR.calculate_set_and_return_tire_force(rear_right_slip_ratio, rear_right_slip_angle, rear_right_normal_force, 0.0);
        
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // Now we can set all the forces and torqes acting on vehicle body -> using object forces_sumed that is in instacne of Forces class for holding summed forces/torqes

        forces_sumed.calculate_and_set_forces_sumed( vx , output_FL, output_FR , output_RL, output_RR);


        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    }


    void Vehicle::update_vehicle_state(vehicle_input input,double dt) {

        // Prepare system 
        prepare_system();
        // Update the vehicle body state based on the calculated forces
        vehicle_body.update_state(forces_sumed, dt);
        
        // For updating angular velocity of tires longitudal forces are needed to know
        double FL_longitudal_force = tire_force_model_FL.get_longitudinal_force();
        double FR_longitudal_force = tire_force_model_FR.get_longitudinal_force();
        double RL_longitudal_force = tire_force_model_RL.get_longitudinal_force();
        double RR_longitudal_force = tire_force_model_RR.get_longitudinal_force();

        // For updating angular velocity of tires 
        double FL_torque = engine_FL.get_torque();
        double FR_torque = engine_FR.get_torque();
        double RL_torque = engine_RL.get_torque();
        double RR_torque = engine_RR.get_torque();

        // Update tires angular velocity based on engine torque and tractive force
        tire_FL.update_omega(FL_torque, FL_longitudal_force, dt);
        tire_FR.update_omega(FR_torque, FR_longitudal_force, dt);
        tire_RL.update_omega(RL_torque, RL_longitudal_force, dt);
        tire_RR.update_omega(RR_torque, RR_longitudal_force, dt);

        // Update steer angle on column
        steer_geometry.update( input.steer, dt);

        // Update torque outputs form engines
        engine_FL.update_torque(input.throttle, dt);
        engine_FR.update_torque(input.throttle, dt);
        engine_RL.update_torque(input.throttle, dt);
        engine_RR.update_torque(input.throttle, dt);

    }
    vehicle_state Vehicle::differential_step_time_dt(vehicle_input current_input, double dt) const {

      Vehicle temp_vehicle(*this);

      temp_vehicle.update_vehicle_state(current_input, dt);

      return this->subtract_this_state_from_other(temp_vehicle.current_state);

    }

    void Vehicle::RK4_integration_step(vehicle_input current_input, double dt) {


    // Prepering four objects for calcuaitng for differentials 

    Vehicle vehicle_temp(*this); // vehicle to be used as a 

    // Step 1 : calculating derivative (k1) in starting state 

    vehicle_state k1 = this->differential_step_time_dt(vehicle_temp.current_state, dt);

    

    // now i am in state 2 -> after updating with k1 derivative through dt/2
 
    vehicle_temp.update_vehicle_state(current_input, dt/2);

    vehicle_state k2 = vehicle_temp.differential_step_time_dt(vehicle_temp.current_state, dt);


    vehicle_temp.update_vehicle_state(current_input, dt);
    vehicle_state k3 = differential_step_time_dt(vehicle_temp.current_state, dt);

    vehicle_temp.update_vehicle_state(current_input, dt);
    vehicle_state k4 = differential_step_time_dt(vehicle_temp.current_state, dt);

    ///

    









    }

} // namespace metzler_model


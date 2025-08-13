#ifndef VEHICLE_H
#define VEHICLE_H

#include "libs.h"
#include "normal_model_forces.h"
#include "forces.h"
#include "tire.h"
#include "engine.h"
#include "tire_force_model.h"
#include "steer_geometry.h"
#include "vehicle_body.h"
#include "params_initializer.h"

namespace metzler_model {

/**
 * @brief Structure representing the full state of the vehicle.
 *
 * Contains position, orientation, velocities, accelerations, engine torques,
 * steering angles, and wheel angular velocities.
 *
 * @param x X position of the vehicle CoG [m]
 * @param y Y position of the vehicle CoG [m]
 * @param yaw Yaw angle of the vehicle [rad]
 * @param vx Longitudinal velocity at CoG [m/s]
 * @param vy Lateral velocity at CoG [m/s]
 * @param yaw_rate Yaw rate [rad/s]
 * @param ax Longitudinal acceleration at CoG [m/s^2]
 * @param ay Lateral acceleration at CoG [m/s^2]
 * @param torque_FL Engine torque front left [Nm]
 * @param torque_FR Engine torque front right [Nm]
 * @param torque_RL Engine torque rear left [Nm]
 * @param torque_RR Engine torque rear right [Nm]
 * @param steer_angle_on_column Steering angle at the column [rad]
 * @param front_left_steer_angle Actual front left wheel steer angle [rad]
 * @param front_right_steer_angle Actual front right wheel steer angle [rad]
 * @param omega_FL Angular velocity of front left wheel [rad/s]
 * @param omega_FR Angular velocity of front right wheel [rad/s]
 * @param omega_RL Angular velocity of rear left wheel [rad/s]
 * @param omega_RR Angular velocity of rear right wheel [rad/s]
 */
struct vehicle_state{
    double x;
    double y;
    double yaw;
    double vx;
    double vy;
    double yaw_rate;
    double ax;
    double ay;
    double torque_FL;
    double torque_FR;
    double torque_RL;
    double torque_RR;
    double steer_angle_on_column;
    double front_left_steer_angle;
    double front_right_steer_angle;
    double omega_FL;
    double omega_FR;
    double omega_RL;
    double omega_RR;
};

/**
 * @brief Zero-initialized constant for vehicle_state.
 */
constexpr vehicle_state zero_vehicle_state{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0
};

/**
 * @brief Structure holding tire slips and forces for all wheels.
 *
 * @param slip_ratio_* Slip ratio for each wheel (FL, FR, RL, RR)
 * @param slip_angle_* Slip angle for each wheel [rad]
 * @param traction_force_* Longitudinal (tractive) force for each wheel [N]
 * @param lateral_force_* Lateral force for each wheel [N]
 */
struct tire_slips_and_forces{
    double slip_ratio_FL;
    double slip_ratio_FR;
    double slip_ratio_RL;
    double slip_ratio_RR;

    double slip_angle_FL;
    double slip_angle_FR;
    double slip_angle_RL;
    double slip_angle_RR;

    double traction_force_FL;
    double traction_force_FR;
    double traction_force_RL;
    double traction_force_RR;

    double lateral_force_FL;
    double lateral_force_FR;
    double lateral_force_RL;
    double lateral_force_RR;
};

/**
 * @brief Structure representing the input to the vehicle.
 *
 * @param throttle Throttle input [0-1]
 * @param steer Steering input [rad]
 */
struct vehicle_input{
    double throttle;
    // double brake; // to be added
    double steer;
};

/**
 * @class Vehicle
 * @brief Class modeling the entire vehicle, including its dynamics, subsystems, and parameters.
 *
 * This class aggregates all major subsystems of the vehicle (body, steering, tires, engines, etc.)
 * and provides methods for state management, system preparation, and integration of vehicle dynamics.
 *
 * @note The Vehicle class is the main interface for simulation and control of the vehicle model.
 */
class Vehicle{

    public:

        /**
         * @brief Current state of the vehicle (position, velocity, yaw, etc.).
         */
        vehicle_state current_state;

        /**
         * @brief Current input to the vehicle (throttle, steer, etc.).
         */
        vehicle_input current_input;

        /**
         * @brief Current tire slips and forces for all wheels.
         */
        tire_slips_and_forces current_slips_and_forces;

        /**
         * @brief Constructor initializing all subsystems using a parameter initializer.
         * @param params_initializer Reference to a ParamsInitializer object providing all necessary parameters.
         */
        Vehicle(ParamsInitializer& params_initializer);

        /**
         * @brief Copying constructor.
         * @param other Reference to another Vehicle object.
         */
        Vehicle(const Vehicle& other);


        void add_other_state_to_this(const Vehicle& other);
        void subtract_this_from_other_state( Vehicle& other) const ;
        void subtract_other_state_from_this( const Vehicle& other);

        // ----------- Setters ------------

        /**
         * @brief Set the current vehicle state.
         * @param state New vehicle state.
         */
        inline void set_state(const vehicle_state& state) { current_state = state; }

        /**
         * @brief Set the current vehicle input.
         * @param input New vehicle input.
         */
        inline void set_input(const vehicle_input& input) { current_input = input; }

        /**
         * @brief Set the current tire slips and forces.
         * @param slips_and_forces New tire slips and forces.
         */
        inline void set_slips_and_forces(const tire_slips_and_forces& slips_and_forces) { current_slips_and_forces = slips_and_forces; }

        /**
         * @brief Set the steering geometry subsystem.
         * @param steer_geom New steering geometry.
         */
        inline void set_steer_geometry(const SteerGeometry& steer_geom) { steer_geometry = steer_geom; }

        /**
         * @brief Set the tire force model for the front left wheel.
         * @param model New tire force model.
         */
        inline void set_tire_force_model_FL(const TireForceModel& model) { tire_force_model_FL = model; }
        inline void set_tire_force_model_FR(const TireForceModel& model) { tire_force_model_FR = model; }
        inline void set_tire_force_model_RL(const TireForceModel& model) { tire_force_model_RL = model; }
        inline void set_tire_force_model_RR(const TireForceModel& model) { tire_force_model_RR = model; }

        /**
         * @brief Set the engine for each wheel.
         * @param engine New engine.
         */
        inline void set_engine_FL(const Engine& engine) { engine_FL = engine; }
        inline void set_engine_FR(const Engine& engine) { engine_FR = engine; }
        inline void set_engine_RL(const Engine& engine) { engine_RL = engine; }
        inline void set_engine_RR(const Engine& engine) { engine_RR = engine; }

        /**
         * @brief Set the tire for each wheel.
         * @param tire New tire.
         */
        inline void set_tire_FL(const Tire& tire) { tire_FL = tire; }
        inline void set_tire_FR(const Tire& tire) { tire_FR = tire; }
        inline void set_tire_RL(const Tire& tire) { tire_RL = tire; }
        inline void set_tire_RR(const Tire& tire) { tire_RR = tire; }

        /**
         * @brief Set the normal forces model.
         * @param forces New normal forces model.
         */
        inline void set_normal_forces(const NormalModelForces& forces) { normal_forces = forces; }

        // ----------- Getters ------------

        /**
         * @brief Get the current vehicle state.
         * @return Current vehicle state.
         */
        inline vehicle_state get_state() const { return current_state; }

        /**
         * @brief Get the current vehicle input.
         * @return Current vehicle input.
         */
        inline vehicle_input get_input() const { return current_input; }

        /**
         * @brief Get the current tire slips and forces.
         * @return Current tire slips and forces.
         */
        inline tire_slips_and_forces get_slips_and_forces() const { return current_slips_and_forces; }

        /**
         * @brief Get the steering geometry subsystem.
         * @return Steering geometry.
         */
        inline SteerGeometry get_steer_geometry() const { return steer_geometry; }

        /**
         * @brief Get the tire force model for each wheel.
         * @return Tire force model.
         */
        inline TireForceModel get_tire_force_model_FL() const { return tire_force_model_FL; }
        inline TireForceModel get_tire_force_model_FR() const { return tire_force_model_FR; }
        inline TireForceModel get_tire_force_model_RL() const { return tire_force_model_RL; }
        inline TireForceModel get_tire_force_model_RR() const { return tire_force_model_RR; }

        /**
         * @brief Get the engine for each wheel.
         * @return Engine.
         */
        inline Engine get_engine_FL() const { return engine_FL; }
        inline Engine get_engine_FR() const { return engine_FR; }
        inline Engine get_engine_RL() const { return engine_RL; }
        inline Engine get_engine_RR() const { return engine_RR; }

        /**
         * @brief Get the tire for each wheel.
         * @return Tire.
         */
        inline Tire get_tire_FL() const { return tire_FL; }
        inline Tire get_tire_FR() const { return tire_FR; }
        inline Tire get_tire_RL() const { return tire_RL; }
        inline Tire get_tire_RR() const { return tire_RR; }

        /**
         * @brief Get the normal forces model.
         * @return Normal forces model.
         */
        inline NormalModelForces get_normal_forces() const { return normal_forces; }

        /**
         * @brief Prepare the vehicle system based on previous state for calculation and integration of further dynamics.
         *
         * This means calculating normal forces, current steering and so on, based on results of the previous integration step.
         * Needed for finding forces for the current integration step.
         */
        void prepare_system();

        /**
         * @brief Integrate the vehicle system based on current input and previous state using Euler step (dt).
         * @param current_input Current input to the vehicle.
         * @param dt Time step [s].
         */
        void integrate_system(vehicle_input current_input, double dt);

        /**
         * @brief Integrate the vehicle system based on current input and previous state using RK4 method.
         * @param current_input Current input to the vehicle.
         * @param dt Time step [s].
         */
        void RK4_integration_step(vehicle_input current_input, double dt);

        /**
         * @brief Reset the vehicle system to initial state.
         */
        inline void reset() { current_state = zero_vehicle_state; }

    private:

        VehicleBody vehicle_body;
        SteerGeometry steer_geometry;

        TireForceModel tire_force_model_FL;
        TireForceModel tire_force_model_FR;
        TireForceModel tire_force_model_RL;
        TireForceModel tire_force_model_RR;

        Engine engine_FL;
        Engine engine_FR;
        Engine engine_RL;
        Engine engine_RR;

        Tire tire_FL;
        Tire tire_FR;
        Tire tire_RL;
        Tire tire_RR;

        NormalModelForces normal_forces;

        Forces forces_sumed;
};

} // namespace
#endif // VEHICLE_H


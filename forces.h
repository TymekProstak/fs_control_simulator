#ifndef FORCES_H
#define FORCES_H

#include "tire_force_model.h"
#include "normal_model_forces.h"

namespace metzler_model {

    /**
     * @brief Structure holding parameters for the Forces model.
     * @param drag_coefficient Aerodynamic drag coefficient.
     * @param downforce_coefficient Aerodynamic downforce coefficient.
     * @param FL_y_position Y position of the front left tire [m].
     * @param FL_x_position X position of the front left tire [m].
     * @param FR_y_position Y position of the front right tire [m].
     * @param FR_x_position X position of the front right tire [m].
     * @param RL_y_position Y position of the rear left tire [m].
     * @param RL_x_position X position of the rear left tire [m].
     * @param RR_y_position Y position of the rear right tire [m].
     * @param RR_x_position X position of the rear right tire [m].
     */
    struct forces_params {
        double drag_coefficient;
        double downforce_coefficient;
        double FL_y_position;
        double FL_x_position;
        double FR_y_position;
        double FR_x_position;
        double RL_y_position;
        double RL_x_position;
        double RR_y_position;
        double RR_x_position;
    };

    /**
     * @brief Structure holding the summed forces and torque.
     * @param fx Force in the x direction [N].
     * @param fy Force in the y direction [N].
     * @param torque Torque around the CoG in the z axis [Nm].
     */
    struct forces_sumed {
        double fx;
        double fy;
        double torque;
    };

    ///***************///// CLASS FORCES ///////*****************///
    class Forces {
    public:
        /**
         * @brief Constructor for the Forces class.
         * @param params Structure with force model parameters.
         */
        Forces(const forces_params& params);

        ///// **** Setters **** ////////////////////

        /**
         * @brief Set the parameters for the force model.
         * @param new_params New parameters.
         */
        inline void set_params(const forces_params& new_params) { params_ = new_params; }

        /**
         * @brief Set the summed forces.
         * @param new_forces New summed forces.
         */
        inline void set_forces_sumed(const forces_sumed& new_forces) { forces_ = new_forces; }

        /**
         * @brief Set the front left tire force.
         * @param new_force New tire force.
         */
        inline void set_FL_tire_force(const tire_force_model_output_in_vehicle_frame& new_force) { FL_tire_force = new_force; }
        /**
         * @brief Set the front right tire force.
         * @param new_force New tire force.
         */
        inline void set_FR_tire_force(const tire_force_model_output_in_vehicle_frame& new_force) { FR_tire_force = new_force; }
        /**
         * @brief Set the rear left tire force.
         * @param new_force New tire force.
         */
        inline void set_RL_tire_force(const tire_force_model_output_in_vehicle_frame& new_force) { RL_tire_force = new_force; }
        /**
         * @brief Set the rear right tire force.
         * @param new_force New tire force.
         */
        inline void set_RR_tire_force(const tire_force_model_output_in_vehicle_frame& new_force) { RR_tire_force = new_force; }

        /**
         * @brief Set the aerodynamic downforce.
         * @param new_downforce New downforce value [N].
         */
        inline void set_aero_downforce(double new_downforce) { areo_downforce = new_downforce; }

        /**
         * @brief Set the aerodynamic drag.
         * @param new_drag New drag value [N].
         */
        inline void set_aero_drag(double new_drag) { areo_drag = new_drag; }

        /**
         * @brief Set the front left tire torque.
         * @param new_torque New torque value [Nm].
         */
        inline void set_FL_tire_torque(double new_torque) { FL_tire_torque = new_torque; }
        /**
         * @brief Set the front right tire torque.
         * @param new_torque New torque value [Nm].
         */
        inline void set_FR_tire_torque(double new_torque) { FR_tire_torque = new_torque; }
        /**
         * @brief Set the rear left tire torque.
         * @param new_torque New torque value [Nm].
         */
        inline void set_RL_tire_torque(double new_torque) { RL_tire_torque = new_torque; }
        /**
         * @brief Set the rear right tire torque.
         * @param new_torque New torque value [Nm].
         */
        inline void set_RR_tire_torque(double new_torque) { RR_tire_torque = new_torque; }

        /////////////////////////////////////////////////////////////////////////

        /////// *************** Getters **************   ///////////////////////

        /**
         * @brief Get the current force model parameters.
         * @return Current parameters.
         */
        inline forces_params get_params() const { return params_; }

        /**
         * @brief Get the summed forces.
         * @return Current summed forces.
         */
        inline forces_sumed get_forces_sumed() const { return forces_; }

        /**
         * @brief Get the front left tire force.
         * @return Tire force.
         */
        inline tire_force_model_output_in_vehicle_frame get_FL_tire_force() const { return FL_tire_force; }
        /**
         * @brief Get the front right tire force.
         * @return Tire force.
         */
        inline tire_force_model_output_in_vehicle_frame get_FR_tire_force() const { return FR_tire_force; }
        /**
         * @brief Get the rear left tire force.
         * @return Tire force.
         */
        inline tire_force_model_output_in_vehicle_frame get_RL_tire_force() const { return RL_tire_force; }
        /**
         * @brief Get the rear right tire force.
         * @return Tire force.
         */
        inline tire_force_model_output_in_vehicle_frame get_RR_tire_force() const { return RR_tire_force; }

        /**
         * @brief Get the aerodynamic downforce.
         * @return Downforce [N].
         */
        inline double get_aero_downforce() const { return areo_downforce; }
        /**
         * @brief Get the aerodynamic drag.
         * @return Drag [N].
         */
        inline double get_aero_drag() const { return areo_drag; }
        /**
         * @brief Get the front left tire torque.
         * @return Torque [Nm].
         */
        inline double get_FL_tire_torque() const { return FL_tire_torque; }
        /**
         * @brief Get the front right tire torque.
         * @return Torque [Nm].
         */
        inline double get_FR_tire_torque() const { return FR_tire_torque; }
        /**
         * @brief Get the rear left tire torque.
         * @return Torque [Nm].
         */
        inline double get_RL_tire_torque() const { return RL_tire_torque; }
        /**
         * @brief Get the rear right tire torque.
         * @return Torque [Nm].
         */
        inline double get_RR_tire_torque() const { return RR_tire_torque; }

        ///////////////////////////////////////////////////////////////////////////

        //////// Force and torque calculations ///////////////////

        ////////////        AERO        /////////////////////////////

        /**
         * @brief Calculate aerodynamic downforce.
         * @param vx Longitudinal velocity [m/s].
         * @return Downforce [N].
         */
        double calculate_aero_downforce(double vx = 0.0) const;

        /**
         * @brief Calculate and set aerodynamic downforce.
         * @param vx Longitudinal velocity [m/s].
         */
        void calculate_and_set_aero_downforce(double vx = 0.0);

        /**
         * @brief Calculate aerodynamic drag.
         * @param vx Longitudinal velocity [m/s].
         * @return Drag [N].
         */
        double calculate_aero_drag(double vx = 0.0) const;

        /**
         * @brief Calculate and set aerodynamic drag.
         * @param vx Longitudinal velocity [m/s].
         */
        void calculate_and_set_aero_drag(double vx = 0.0);

        ////////////    Tire traction       ////////////////////////////

        /**
         * @brief Calculate front left tire torque.
         * @param new_force Tire force structure.
         * @return Torque [Nm].
         */
        double calculate_FL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) const;
        /**
         * @brief Calculate front right tire torque.
         * @param new_force Tire force structure.
         * @return Torque [Nm].
         */
        double calculate_FR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) const;
        /**
         * @brief Calculate rear left tire torque.
         * @param new_force Tire force structure.
         * @return Torque [Nm].
         */
        double calculate_RL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) const;
        /**
         * @brief Calculate rear right tire torque.
         * @param new_force Tire force structure.
         * @return Torque [Nm].
         */
        double calculate_RR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force) const;

        /**
         * @brief Calculate and set front left tire torque.
         * @param new_force Tire force structure.
         */
        void calculate_and_set_FL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force);
        /**
         * @brief Calculate and set front right tire torque.
         * @param new_force Tire force structure.
         */
        void calculate_and_set_FR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force);
        /**
         * @brief Calculate and set rear left tire torque.
         * @param new_force Tire force structure.
         */
        void calculate_and_set_RL_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force);
        /**
         * @brief Calculate and set rear right tire torque.
         * @param new_force Tire force structure.
         */
        void calculate_and_set_RR_tire_torque(const tire_force_model_output_in_vehicle_frame& new_force);

        //////// Output Function //////////////////

        /**
         * @brief Calculate summed forces and torque for the vehicle.
         * @param vx Longitudinal velocity [m/s].
         * @param FL_force Front left tire force.
         * @param FR_force Front right tire force.
         * @param RL_force Rear left tire force.
         * @param RR_force Rear right tire force.
         * @return Summed forces and torque.
         */
        forces_sumed calculate_forces_sumed(
            double vx = 0.0,
            tire_force_model_output_in_vehicle_frame& FL_force,
            tire_force_model_output_in_vehicle_frame& FR_force,
            tire_force_model_output_in_vehicle_frame& RL_force,
            tire_force_model_output_in_vehicle_frame& RR_force
        ) const;

        /**
         * @brief Calculate and set summed forces and torque for the vehicle.
         * @param vx Longitudinal velocity [m/s].
         * @param FL_force Front left tire force.
         * @param FR_force Front right tire force.
         * @param RL_force Rear left tire force.
         * @param RR_force Rear right tire force.
         */
        void calculate_and_set_forces_sumed(
            double vx = 0.0,
            tire_force_model_output_in_vehicle_frame& FL_force,
            tire_force_model_output_in_vehicle_frame& FR_force,
            tire_force_model_output_in_vehicle_frame& RL_force,
            tire_force_model_output_in_vehicle_frame& RR_force
        );

    private:

        forces_params params_;
        forces_sumed forces_;

        tire_force_model_output_in_vehicle_frame RR_tire_force;
        tire_force_model_output_in_vehicle_frame FL_tire_force;
        tire_force_model_output_in_vehicle_frame FR_tire_force;
        tire_force_model_output_in_vehicle_frame RL_tire_force;

        double areo_downforce;
        double areo_drag;
        double FL_tire_torque;
        double FR_tire_torque;
        double RL_tire_torque;
        double RR_tire_torque;

    };

} // namespace metzler_model

#endif // FORCES_H
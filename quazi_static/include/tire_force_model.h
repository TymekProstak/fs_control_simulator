#ifndef TIRE_FORCE_MODEL_H
#define TIRE_FORCE_MODEL_H

#include "libs.h"
#include "tire.h"

namespace metzler_model {

    /**
     * @brief Parameters for the tire force model (Simplified Pacejka).
     * @param D_lat Lateral force peak factor (treated as friction coefficient for now).
     * @param C_lat Lateral shape factor.
     * @param B_lat Lateral stiffness factor.
     * @param D_long Longitudinal force peak factor (treated as friction coefficient for now).
     * @param C_long Longitudinal shape factor.
     * @param B_long Longitudinal stiffness factor.
     * @param E_long Rolling resistance .
     */
    struct tire_forces_model_params {
        // Lateral parameters
        double D_lat;
        double C_lat;
        double B_lat;

        // Longitudinal parameters
        double D_long;
        double C_long;
        double B_long;
        double E_long;
    };

    /**
     * @brief Input structure for the tire force model.
     * @param slip_ratio Tire slip ratio.
     * @param slip_angle Tire slip angle [rad].
     * @param normal_force Normal force on the tire [N].
     * @param steer_angle Steering angle of the tire [rad].
     */
    struct tire_force_model_input {
        double slip_ratio;
        double slip_angle;
        double normal_force;
        double steer_angle;
    };

    /**
     * @brief Output structure for the tire force model.
     * @param longitudinal_force Longitudinal force [N].
     * @param lateral_force Lateral force [N].
     */
    struct tire_force_model_output {
        double longitudinal_force;
        double lateral_force;
    };

    /**
     * @brief Tire force output in the vehicle frame.
     * @param fx Force in the vehicle X direction [N].
     * @param fy Force in the vehicle Y direction [N].
     */
    struct tire_force_model_output_in_vehicle_frame {
        double fx;
        double fy;
    };

    /**
     * @brief Class implementing the simplified Pacejka tire force model.
     */
    class TireForceModel {
    public:
        /**
         * @brief Constructor.
         * @param params Tire force model parameters.
         */
        TireForceModel(const tire_forces_model_params& params);

        // ***** Setters *****

        /**
         * @brief Set the tire force model parameters.
         * @param params New parameters.
         */
        inline void set_params(const tire_forces_model_params& params) { params_ = params; }

        /**
         * @brief Set the input for the tire force model.
         * @param input New input.
         */
        inline void set_input_(const tire_force_model_input& input) { input_ = input; }

        /**
         * @brief Set the tire force model output.
         * @param output New output.
         */
        inline void set_tire_force_model_output(const tire_force_model_output& output) { output_ = output; }

        /**
         * @brief Set the tire force output in the vehicle frame.
         * @param output_in_vehicle_frame New output in vehicle frame.
         */
        inline void set_output_in_vehicle_frame(const tire_force_model_output_in_vehicle_frame& output_in_vehicle_frame) { output_in_vehicle_frame_ = output_in_vehicle_frame; }

        // ***** Getters *****

        /**
         * @brief Get the tire force model parameters.
         * @return Current parameters.
         */
        tire_forces_model_params get_params() const { return params_; }

        /**
         * @brief Get the current input.
         * @return Current input.
         */
        tire_force_model_input get_input() const { return input_; }

        /**
         * @brief Get the current output.
         * @return Current output.
         */
        tire_force_model_output get_output() const { return output_; }

        /**
         * @brief Get the output in the vehicle frame.
         * @return Output in vehicle frame.
         */
        tire_force_model_output_in_vehicle_frame get_output_in_vehicle_frame() const { return output_in_vehicle_frame_; }

        /**
         * @brief Get the current tractive (longitudinal) force.
         * @return Tractive force [N].
         */
        double get_tractive_force() const { return output_.longitudinal_force; }

        // ***** Tire force calculations *****

        /**
         * @brief Calculate tire force using slip ratio, slip angle, normal force, and steer angle.
         * @param slip_ratio Tire slip ratio.
         * @param slip_angle Tire slip angle [rad].
         * @param normal_force Normal force on the tire [N].
         * @param steer_angle Steering angle of the tire [rad].
         * @return Tire force output.
         */
        tire_force_model_output calculate_tire_force(double slip_ratio = 0.0, double slip_angle = 0.0, double normal_force = 0.0, double steer_angle = 0.0) const;

        /**
         * @brief Calculate tire force using an input structure.
         * @param input Tire force model input.
         * @return Tire force output.
         */
        tire_force_model_output calculate_tire_force(const tire_force_model_input& input) const;

        /**
         * @brief Calculate tire force in the vehicle frame using slip ratio, slip_angle, normal_force, and steer_angle.
         * @param slip_ratio Tire slip ratio.
         * @param slip_angle Tire slip angle [rad].
         * @param normal_force Normal force on the tire [N].
         * @param steer_angle Steering angle of the tire [rad].
         * @return Tire force output in vehicle frame.
         */
        tire_force_model_output_in_vehicle_frame calculate_tire_force_in_vehicle_frame(double slip_ratio = 0.0, double slip_angle = 0.0, double normal_force = 0.0, double steer_angle = 0.0) const;

        /**
         * @brief Calculate tire force in the vehicle frame using an input structure.
         * @param input Tire force model input.
         * @return Tire force output in vehicle frame.
         */
        tire_force_model_output_in_vehicle_frame calculate_tire_force_in_vehicle_frame(const tire_force_model_input& input) const;

        /**
         * @brief Calculate and set tire force using an input structure.
         * @param input Tire force model input.
         */
        void calculate_and_set_tire_force(const tire_force_model_input& input);

        /**
         * @brief Calculate and set tire force using slip ratio, slip angle, normal force, and steer angle.
         * @param slip_ratio Tire slip ratio.
         * @param slip_angle Tire slip angle [rad].
         * @param normal_force Normal force on the tire [N].
         * @param steer_angle Steering angle of the tire [rad].
         */
        void calculate_and_set_tire_force(double slip_ratio = 0.0, double slip_angle = 0.0, double normal_force = 0.0, double steer_angle = 0.0);

        /**
         * @brief Calculate and set tire force in the vehicle frame using slip ratio, slip angle, normal force, and steer angle.
         * @param slip_ratio Tire slip ratio.
         * @param slip_angle Tire slip angle [rad].
         * @param normal_force Normal force on the tire [N].
         * @param steer_angle Steering angle of the tire [rad].
         */
        void calculate_and_set_tire_force_in_vehicle_frame(double slip_ratio = 0.0, double slip_angle = 0.0, double normal_force = 0.0, double steer_angle = 0.0);

        /**
         * @brief Calculate and set tire force in the vehicle frame using an input structure.
         * @param input Tire force model input.
         */
        void calculate_and_set_tire_force_in_vehicle_frame(const tire_force_model_input& input);

        /**
         * @brief Calculate, set, and return tire force using an input structure.
         * @param input Tire force model input.
         * @return Tire force output.
         */
        tire_force_model_output calculate_set_and_return_tire_force(const tire_force_model_input& input);

        /**
         * @brief Calculate, set, and return tire force using slip ratio, slip angle, normal force, and steer angle.
         * @param slip_ratio Tire slip ratio.
         * @param slip_angle Tire slip angle [rad].
         * @param normal_force Normal force on the tire [N].
         * @param steer_angle Steering angle of the tire [rad].
         * @return Tire force output.
         */
        tire_force_model_output calculate_set_and_return_tire_force(double slip_ratio = 0.0, double slip_angle = 0.0, double normal_force = 0.0, double steer_angle = 0.0);

        /**
         * @brief Calculate, set, and return tire force in the vehicle frame using an input structure.
         * @param input Tire force model input.
         * @return Tire force output in vehicle frame.
         */
        tire_force_model_output_in_vehicle_frame calculate_set_and_return_tire_force_in_vehicle_frame(const tire_force_model_input& input);

        /**
         * @brief Calculate, set, and return tire force in the vehicle frame using slip ratio, slip angle, normal force, and steer_angle.
         * @param slip_ratio Tire slip ratio.
         * @param slip_angle Tire slip angle [rad].
         * @param normal_force Normal force on the tire [N].
         * @param steer_angle Steering angle of the tire [rad].
         * @return Tire force output in vehicle frame.
         */
        tire_force_model_output_in_vehicle_frame calculate_set_and_return_tire_force_in_vehicle_frame(double slip_ratio = 0.0, double slip_angle = 0.0, double normal_force = 0.0, double steer_angle = 0.0);

    private:
        tire_forces_model_params params_;
        tire_force_model_input input_;
        tire_force_model_output output_;
        tire_force_model_output_in_vehicle_frame output_in_vehicle_frame_;
    };

} // namespace metzler_model

#endif // TIRE_FORCE_MODEL_H

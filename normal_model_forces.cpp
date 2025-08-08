#include "normal_model_forces.h"

namespace metzler_model {

///***************///// KLASA NORMALMODEL DEFINICJE ///////*****************///

NormalModelForces::NormalModelForces(const normal_model_forces_params& params)
    : params_(params), forces_{} {}

normal_model_forces_output NormalModelForces::calculate_normal_forces(double vx, double vy, double ax, double ay) const {
    // Calculate the normal forces acting on each wheel
    normal_model_forces_output forces;


    /// for model of finding the distribution of normal forces refrence to  https://kktse.github.io/jekyll/update/2021/05/12/simplied-lateral-load-transfer-analysis.html ///

    ///***************///// STRUKTURY POMOCNICZE ///////*****************///
    normal_model_forces_output static_forces;
    normal_model_forces_output longitudal_mass_transfer_forces;
    normal_model_forces_output aerodynamic_forces;
    normal_model_forces_output link_load_transfer_forces;
    normal_model_forces_output roll_stiffness_load_transfer;

    double mass = params_.mass;
    double gravity = params_.gravity;

    double wheelbase = params_.wheelbase;
    double wheel_distance_front = params_.wheel_distance_front;
    double wheel_distance_rear = params_.wheel_distance_rear;
    double wheel_distance_avg = params_.wheel_distance_avg;

    double lf = params_.lf;
    double lr = params_.lr;

    double h_cg = params_.h_cg;
    double h_roll_f = params_.h_roll_f;
    double h_roll_r = params_.h_roll_r;

    double roll_stiffness_front = params_.front_roll_stiffness;
    double roll_stiffness_rear = params_.rear_roll_stiffness;
    double roll_stiffness_sum = roll_stiffness_front + roll_stiffness_rear;

    double h_prim_front = params_.h_cg - h_roll_f;
    double h_prim_rear = params_.h_cg - h_roll_r;

    double mass_front = mass * (lf / (lf + lr));
    double mass_rear = mass * (lr / (lf + lr));

    double downforce_coefficient = params_.downforce_coefficient;
    double drag_coefficient = params_.drag_coefficient;
    double pressure_center_height = params_.pressure_center_height;

    ///***************///// STATIC FORCES ///////*****************///
    static_forces.front_left_force = 0.5 * mass * gravity * (lf / (lf + lr));
    static_forces.front_right_force = 0.5 * mass * gravity * (lf / (lf + lr));
    static_forces.rear_left_force = 0.5 * mass * gravity * (lr / (lf + lr));
    static_forces.rear_right_force = 0.5 * mass * gravity * (lr / (lf + lr));

    ///***************///// LONGITUDAL MASS TRANSFER ///////*****************///
    double longitudal_mass_transfer = mass * h_cg / (lf + lr) * ax * 0.5;

    longitudal_mass_transfer_forces.front_left_force = -longitudal_mass_transfer;
    longitudal_mass_transfer_forces.front_right_force = -longitudal_mass_transfer;
    longitudal_mass_transfer_forces.rear_left_force = longitudal_mass_transfer;
    longitudal_mass_transfer_forces.rear_right_force = longitudal_mass_transfer;

    ///***************///// LINK LOAD TRANSFER ///////*****************///
    double link_load_transfer_front = mass_front * h_roll_f * ay / wheel_distance_front;
    double link_load_transfer_rear = mass_rear * h_roll_r * ay / wheel_distance_rear;

    link_load_transfer_forces.front_left_force = -link_load_transfer_front;
    link_load_transfer_forces.front_right_force = link_load_transfer_front;
    link_load_transfer_forces.rear_left_force = -link_load_transfer_rear;
    link_load_transfer_forces.rear_right_force = link_load_transfer_rear;

    ///***************///// ROLL STIFFNESS LOAD TRANSFER ///////*****************///
    double roll_transfer_front = (mass_front * h_prim_front + mass_rear * h_prim_rear) / wheel_distance_front * roll_stiffness_front / roll_stiffness_sum;
    double roll_transfer_rear = (mass_front * h_prim_front + mass_rear * h_prim_rear) / wheel_distance_rear * roll_stiffness_rear / roll_stiffness_sum;

    roll_stiffness_load_transfer.front_left_force = -ay * roll_transfer_front;
    roll_stiffness_load_transfer.front_right_force = ay * roll_transfer_front;
    roll_stiffness_load_transfer.rear_left_force = -ay * roll_transfer_rear;
    roll_stiffness_load_transfer.rear_right_force = ay * roll_transfer_rear;

    ///***************///// AERODYNAMIC FORCES ///////*****************///

    /// Downforce part
    double aerodynamic_downforce = params_.downforce_coefficient * (vx * vx) *0.5;

    aerodynamic_forces.front_left_force = aerodynamic_downforce * lf / (lf + lr);
    aerodynamic_forces.front_right_force = aerodynamic_downforce * lf / (lf + lr);
    aerodynamic_forces.rear_left_force = aerodynamic_downforce * lr / (lf + lr);
    aerodynamic_forces.rear_right_force = aerodynamic_downforce * lr / (lf + lr);

    /// Drag - pitch part

    double aerodynamic_drag_pitch = params_.drag_coefficient * (vx * vx) * pressure_center_height / (lf + lr) * 0.5 ;
    aerodynamic_forces.front_left_force -= aerodynamic_drag_pitch;
    aerodynamic_forces.front_right_force -= aerodynamic_drag_pitch;
    aerodynamic_forces.rear_left_force += aerodynamic_drag_pitch;
    aerodynamic_forces.rear_right_force += aerodynamic_drag_pitch;

    ///***************///// SUMA SIŁ ///////*****************///
    forces.front_left_force = static_forces.front_left_force + longitudal_mass_transfer_forces.front_left_force + aerodynamic_forces.front_left_force + link_load_transfer_forces.front_left_force + roll_stiffness_load_transfer.front_left_force;
    forces.front_right_force = static_forces.front_right_force + longitudal_mass_transfer_forces.front_right_force + aerodynamic_forces.front_right_force + link_load_transfer_forces.front_right_force + roll_stiffness_load_transfer.front_right_force;
    forces.rear_left_force = static_forces.rear_left_force + longitudal_mass_transfer_forces.rear_left_force + aerodynamic_forces.rear_left_force + link_load_transfer_forces.rear_left_force + roll_stiffness_load_transfer.rear_left_force;
    forces.rear_right_force = static_forces.rear_right_force + longitudal_mass_transfer_forces.rear_right_force + aerodynamic_forces.rear_right_force + link_load_transfer_forces.rear_right_force + roll_stiffness_load_transfer.rear_right_force;

    return forces;
}

void NormalModelForces::calculate_and_set_normal_forces(double vx, double vy, double ax, double ay) {
    forces_ = calculate_normal_forces(vx, vy, ax, ay);
}

normal_model_forces_output NormalModelForces::calculate_set_and_return_normal_forces(double vx, double vy, double ax, double ay) {
    forces_ = calculate_normal_forces(vx, vy, ax, ay);
    return forces_;
}

} // namespace

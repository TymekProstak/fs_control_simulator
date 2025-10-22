#include "simulation_lem_ros_node.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace lem_dynamics_sim_ {

// ====== Konstruktor / inicjalizacja ======
Simulation_lem_ros_node::Simulation_lem_ros_node(ros::NodeHandle& nh,
                                                 const std::string& param_file,
                                                 const std::string& track_file,
                                                 const std::string& /*log_file*/)
: nh_(nh)
{
    // 1) Parametry i tor
    P_.loadFromFile(param_file);
    state_.setZero();
    track_global_ = load_track_from_file(track_file);

    // 2) PID / sterowanie – reset
    pid_prev_I_ = 0.0;
    pid_prev_error_ = 0.0;
    target_wheel_speed_ = 0.0;
    pid_speed_out_ = 0.0;
    pid_omega_actual_ = 0.0;
    torque_command_ = 0.0;
    steer_command_ = 0.0;

    // 3) Interwały krokowe (po wczytaniu P_)
    compute_step_intervals_from_params_();

    // 4) ROS I/O (tematy wstaw wg swojego projektu)
    sub_control_ = nh_.subscribe<std_msgs::Float64MultiArray>(
        "/lem/control", 1, &Simulation_lem_ros_node::dv_control_callback, this);

    pub_ins_   = nh_.advertise<std_msgs::Float64MultiArray>("/lem/ins", 1);
    pub_cones_ = nh_.advertise<std_msgs::Float64MultiArray>("/lem/cones", 1);

    // 5) Kolejka kamery pusta na start
    camera_queue_.clear();
}

// ====== Interfejs publiczny ======
void Simulation_lem_ros_node::step() {
    // 1) Zastosuj wejścia z opóźnieniem (jeśli nadszedł czas)
    apply_delayed_inputs_if_due_();

    // 2) Odczyty czujników wg harmonogramu
    read_wheel_encoder_if_due_();
    read_ins_if_due_();
    shoot_camera_or_enqueue_if_due_();

    // 3) Aktualizacja PID (jeśli pracujemy w trybie speed)
    update_pid_if_due_();

    // 4) Publikacje ramek, które „dojrzały” (gotowe po opóźnieniu obliczeń)
    publish_ready_camera_frames_from_queue_();
    

    // step fizyki układu na razie euler do dyspozycji jest też runne kutta

    euler_sim_timestep(state_,Input(torque_command_,steer_command_) P );
    // 5) Inkrement kroku
    ++step_number_;
}

void Simulation_lem_ros_node::step_speed_command(const DV_control_input& u) {
    // Zapamiętaj żądane wejście – zastosujemy z opóźnieniami
    last_input_requested_ = u;

    // Ustaw „deadline” zastosowania obu wejść (jeśli nie czekamy już na istniejący deadline)
    if (step_number_to_apply_steer_input_ <= step_number_)
        step_number_to_apply_steer_input_  = step_number_ + step_of_steer_input_application_;

    if (step_number_to_apply_torque_input_ <= step_number_)
        step_number_to_apply_torque_input_ = step_number_ + step_of_torque_input_application_;

    
    
}

State Simulation_lem_ros_node::get_state() const { return state_; }
ParamBank Simulation_lem_ros_node::get_parameters() const { return P_; }
int Simulation_lem_ros_node::get_step_number() const { return step_number_; }

// ====== ROS callback (np. u = [torque, steer]) ======
void Simulation_lem_ros_node::dv_control_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 3) return;
    DV_control_input u
    u.torque = msg->data[0];
    u.steer  = msg->data[1];
    u.torque_mode = msg->data[2];
    last_input_requested_ = u;

}

// ====== Pomocnicze ======
void Simulation_lem_ros_node::compute_step_intervals_from_params_() {
    const double dt = P_.get("simulation_time_step");

    // Częstotliwości → kroki
    step_of_camera_shoot_          = std::max(1, (int)std::round(1.0 / P_.get("frames_per_second") / dt));
    step_of_wheel_encoder_reading_ = std::max(1, (int)std::round(P_.get("wheel_encoder_reading_time_step") / dt));
    step_of_ins_reading_           = std::max(1, (int)std::round(1.0 / P_.get("ins_frequancy") / dt));

    step_of_torque_input_application_ = std::max(1, (int)std::round(P_.get("torque_input_delay") / dt));
    step_of_steer_input_application_  = std::max(1, (int)std::round(P_.get("steer_input_delay") / dt));

    step_number_pid_update_period_    = std::max(1, (int)std::round(P_.get("pid_time_step") / dt));
}

void Simulation_lem_ros_node::apply_delayed_inputs_if_due_() {
    // Sterowanie skrętem
    if (step_number_to_apply_steer_input_ > 0 && step_number_ % step_of_steer_input_application_ == 0_) {
        steer_command_ = last_input_requested_.steer;
    }
    // Sterowanie momentem
    if (step_number_to_apply_torque_input_ > 0 && step_number_ % step_of_torque_input_application_ ==  0) {
        torque_command_ = last_input_requested_.torque;
        torque_mode_    =  last_input_requested_.torque_mode; // 0 - torque , 1 - speed
    }
}

void Simulation_lem_ros_node::read_wheel_encoder_if_due_() {
    if (step_of_wheel_encoder_reading_ <= 0) return;
    if (step_number_ % step_of_wheel_encoder_reading_ != 0) return;

    // Przykładowy „odczyt” – uśrednienie tylnych kół (dostosuj do modelu)
    pid_omega_actual_ = (state_.omega_rr + state_.omega_rl) * 0.5;
    // TODO: jeżeli są opóźnienia/latencje enkodera, dodaj własną kolejkę analogiczną do kamery
}

void Simulation_lem_ros_node::read_ins_if_due_() {
    if (step_of_ins_reading_ <= 0) return;
    if (step_number_ % step_of_ins_reading_ != 0) return;

    // Dodaj szumy pomiarowe INS i proste pochodne „z różniczki”
    const double f_ins = P_.get("ins_frequancy");
    const double Ts    = 1.0 / std::max(1e-6, f_ins);

    ins_data_to_be_published_.x   = state_.x   + P_.get("ins_x_noise")       * random_noise_generator_();
    ins_data_to_be_published_.y   = state_.y   + P_.get("ins_y_noise")       * random_noise_generator_();
    ins_data_to_be_published_.yaw = state_.yaw + P_.get("ins_yaw_noise")     * random_noise_generator_();
    ins_data_to_be_published_.vx  = (ins_data_to_be_published_.x - last_ins_data_already_published_.x) / Ts;
    ins_data_to_be_published_.vy  = (ins_data_to_be_published_.y - last_ins_data_already_published_.y) / Ts;
    ins_data_to_be_published_.yaw_rate =
        state_.yaw_rate + P_.get("ins_yaw_rate_noise") * random_noise_generator_();

    // Publikacja bez opóźnienia (jeśli chcesz zasymulować latencję INS – dodaj kolejkę)
    publish_ins_(ins_data_to_be_published_);
    last_ins_data_already_published_ = ins_data_to_be_published_;
}

void Simulation_lem_ros_node::shoot_camera_or_enqueue_if_due_() {
    if (step_of_camera_shoot_ <= 0) return;
    if (step_number_ % step_of_camera_shoot_ != 0) return;

    // Generuj „surowe” detekcje stożków w układzie kamery
    Track detection = get_cone_detections_in_camera_frame(state_, track_global_, P_);

    // Oszacuj koszt obliczeń i wyznacz krok gotowości
    // (np. P_.get("camera_processing_time") w sekundach → na kroki)
    // tutaj t student na szacowanie czasu obliczeń na podstawie dancyh od janka

    CameraTask task;
    task.ready_step = step_number_ + processing_steps;
    task.frame      = std::move(detection);

    // Kolejka maks 3 elementy: jeśli pełna — zdejmij najstarszą (upuszczamy ramkę)
    if ((int)camera_queue_.size() >= 3) {
        camera_queue_.pop_front();
    }
    camera_queue_.push_back(std::move(task));
}

void Simulation_lem_ros_node::publish_ready_camera_frames_from_queue_() {
    // Publikuj wszystkie ramki z kolejki, które są już „gotowe”
    while (!camera_queue_.empty() && camera_queue_.front().ready_step <= step_number_) {
        const auto& task = camera_queue_.front();
        publish_cones_(task.frame);
        camera_queue_.pop_front(); // po wysłaniu usuwamy z kolejki
    }
}

void Simulation_lem_ros_node::update_pid_if_due_() {
    if (torque_mode_ == 0) return;  // w trybie „torque” PID może być nieużywany
    if (step_number_pid_update_period_ <= 0) return;
    if (step_number_ % step_number_pid_update_period_ != 0) return;

    // TODO: wstaw właściwą logikę PID (P/I/D, anti-windup, itd.)
    // przykład szkicu:
    // double error = target_wheel_speed_ - pid_omega_actual_;
    // pid_prev_I_ += error * (step_number_pid_update_period_ * P_.get("simulation_time_step"));
    // double u = Kp*error + Ki*pid_prev_I_ + Kd*(error - pid_prev_error_)/Ts_pid;
    // pid_prev_error_ = error;
    // pid_speed_out_ = u;
    // torque_command_ = saturate(u, -Tmax, Tmax);
}

double Simulation_lem_ros_node::random_noise_generator_() const {
    // Prosty białoszum ~ N(0,1)
    static thread_local std::mt19937 rng{std::random_device{}()};
    static thread_local std::normal_distribution<double> N01(0.0, 1.0);
    return N01(rng);
}

// ====== Publikacje (szkielet – dostosuj typy wiadomości/tematy) ======
void Simulation_lem_ros_node::publish_ins_(const INS_data& ins) {
    std_msgs::Float64MultiArray msg;
    msg.data = {ins.x, ins.y, ins.yaw, ins.vx, ins.vy, ins.yaw_rate};
    pub_ins_.publish(msg);
}

void Simulation_lem_ros_node::publish_cones_(const Track& cones) {
    // Przykładowe opakowanie do prostego wektora double (x1,y1,x2,y2,...)
    std_msgs::Float64MultiArray msg;
    msg.data.reserve(cones.cones.size() * 2);
    for (const auto& c : cones.cones) {
        msg.data.push_back(c.x);
        msg.data.push_back(c.y);
    }
    pub_cones_.publish(msg);
}

} // namespace lem_dynamics_sim_

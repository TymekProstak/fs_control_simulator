#include "sim_loop.hpp"

namespace lem_dynamics_sim_ {

// ====== Konstruktor / inicjalizacja ======
Simulation_lem_ros_node::Simulation_lem_ros_node(ros::NodeHandle& nh,
                                                 const std::string& param_file,
                                                 const std::string& track_file,
                                                 const std::string& log_file )
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
    torque_command_ = 0.0; // may represent torque or speed command depending on mode
    torque_command_to_invert_ = 0.0;
    steer_command_ = 0.0;

    // 3) Interwały krokowe (po wczytaniu P_)
    compute_step_intervals_from_params_();

    // 4) ROS I/O (tematy wstaw wg swojego projektu)
    sub_control_ = nh_.subscribe<dv::interfaces::Control>(
        "/dv_board/control", 1, &Simulation_lem_ros_node::dv_control_callback, this, ros::TransportHints().tcpNoDelay());

    pub_ins_   = nh_.advertise<nav_msgs::Odometry >("ins/pose", 1);
    pub_cones_ = nh_.advertise<dv_interfaces::Cones >("/dv_cone_detector/cones", 1);

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
    if( torque_mode == 1) update_pid_if_due_(); // 1 - speed , 0 - torque

    else {
        torque_command_to_invert_ = torque_command_; // w trybie torque bez zmian
    }

    // 4) Publikacje ramek, które „dojrzały” (gotowe po opóźnieniu obliczeń)
    publish_ready_camera_frames_from_queue_();
    

    // step fizyki układu na razie euler do dyspozycji jest też runne kutta

    euler_sim_timestep(state_,Input(torque_command_to_invert_,steer_command_) P );

    // 5) Inkrement kroku
    ++step_number_;
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
        torque_command_ = double(last_input_requested_.torque);
        torque_mode_    =  int(last_input_requested_.torque_mode); // 0 - torque , 1 - speed
        
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
    ins_data_to_be_published_.yaw_rate = state_.yaw_rate + P_.get("ins_yaw_rate_noise") * random_noise_generator_();
    // Publikacja bez opóźnienia (jeśli chcesz zasymulować latencję INS – dodaj kolejkę)
    publish_ins_(ins_data_to_be_published_);
    last_ins_data_already_published_ = ins_data_to_be_published_;
}

void Simulation_lem_ros_node::shoot_camera_or_enqueue_if_due_() {
    if (step_of_camera_shoot_ <= 0) return;
    if (step_number_ % step_of_camera_shoot_ != 0) return;

    // 1) „Zrób zdjęcie" i policz detekcje stożków w układzie kamery
    Track detection = get_cone_detections_in_camera_frame(state_, track_global_, P_);

    // 2) Oszacuj koszt obliczeń wizji z t-Studenta(ν=6) na podstawie μ i var z P_
    const double dt               = P_.get("simulation_time_step");
    const double vision_exec_time = sample_vision_exec_time_();            // [s]
    const int    processing_steps = std::max(0, (int)std::round(vision_exec_time / dt));

    // 3) Zbuduj zadanie i wstaw do kolejki (max 3 elementy)
    CameraTask task;
    task.ready_step = step_number_ + processing_steps;
    task.frame      = std::move(detection);

    if ((int)camera_queue_.size() >= 3) {
        camera_queue_.pop_front(); // upuść najstarszą ramkę
        timestamp_queue_.pop_front(); // upuść odpowiadający timestamp
    }

    camera_queue_.push_back(std::move(task));
    timestamp_queue_.push_back(ros::Time::now());
}

void Simulation_lem_ros_node::publish_ready_camera_frames_from_queue_() {
    // Publikuj wszystkie ramki z kolejki, które są już „gotowe"
    while (!camera_queue_.empty() && camera_queue_.front().ready_step <= step_number_) {
        const auto& task = camera_queue_.front();
        const auto& timestamp = timestamp_queue_.front();
        publish_cones_(task.frame, timestamp);
        camera_queue_.pop_front(); // po wysłaniu usuwamy z kolejki
        timestamp_queue_.pop_front();
    }
}
void Simulation_lem_ros_node::update_pid_if_due_() {
    if (torque_mode_ == 0) return;  // w trybie „torque” PID może być nieużywany
    if (step_number_pid_update_period_ <= 0) return;
    if (step_number_ % step_number_pid_update_period_ != 0) return;

    
    target_wheel_speed_ = torque_command_; // tak jest w trybie speed póki co

    // głupi pid jak na dv boardzie powiny być możę feedforwqad + antiwidnaup at least ale to do rozwoju sytemu
    double error = target_wheel_speed_ - pid_omega_actual_ * P.get("R");
    pid_prev_I_ += error * (step_number_pid_update_period_ * P_.get("pid_time_step"));
    double u = P.get("pid_p")*error + P.get("pid_i")*pid_prev_I_ + P.get("pid_d")*(error - pid_prev_error_)/P.get("pid_time_step");
    pid_prev_error_ = error;
    torque_command_to_invert_ = u; 
   
}

double Simulation_lem_ros_node::random_noise_generator_() const {
    // Prosty białoszum ~ N(0,1)
    static thread_local std::mt19937 rng{std::random_device{}()};
    static thread_local std::normal_distribution<double> N01(0.0, 1.0);
    return N01(rng);
}

// ====== Publikacje (szkielet – dostosuj typy wiadomości/tematy) ======

void Simulation_lem_ros_node::publish_ins_(const INS_data& ins){
    nav_msgs::Odometry odom_msg{}; // wpisuje we wszytkie pola zera domyślnie

    // Nagłówek: czas i frame_id
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "map";          // globalny układ odniesienia
    odom_msg.child_frame_id  = "bolide_CoG";    // lokalny układ pojazdu

    // --- Pozycja ---
    odom_msg.pose.pose.position.x = ins.x;
    odom_msg.pose.pose.position.y = ins.y;
    odom_msg.pose.pose.position.z = 0.0;

    // --- Orientacja (z yaw) ---
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, ins.yaw);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // --- Prędkości ---
    odom_msg.twist.twist.linear.x  = ins.vx;
    odom_msg.twist.twist.linear.y  = ins.vy;
    odom_msg.twist.twist.linear.z  = 0.0;
    odom_msg.twist.twist.angular.z = ins.yaw_rate;


    // Publikacja
    pub_ins_.publish(odom_msg);
}


void Simulation_lem_ros_node::publish_cones_(const Track& cones, ros::Time timestamp){

    dv_interfaces::Cones cones_msg;

    // Nagłówek
    cones_msg.header.stamp = timestamp;
    cones_msg.header.frame_id = "camera_base"; // układ kamery

    // Wypełnienie detekcji stożków
    for (const auto& cone : cones.cones) {
        dv_interfaces::Cone cone_msg;
        cone_msg.confidence = 1.0; // na razie pełne zaufanie i tak nie jest używane
        cone_msg.x = float32(cone.x);
        cone_msg.y = float32(cone.y);
        cone_msg.z = float32(cone.z);
        cone_msg.distance_uncertainty = cone.distance;
        cone_msg.class_name = cone.color; // eg yellow , blue
        cones_msg.cones.push_back(cone_msg);



    }

    // Publikacja
    pub_cones_.publish(cones_msg);
   
}

double Simulation_lem_ros_node::sample_vision_exec_time_() const
{
    // Parametry z ParamBank (sekundy i sekundy^2)
    const double mu   = P_.get("vision.mean_time_of_vision_execuction");   // średni czas [s]
    const double var  = P_.get("vision.var_of_vision_time_execution");     // wariancja [s^2]

    // t-Student z ν=6 (wg Twojej prośby); dla ν>2 wariancja t wynosi ν/(ν-2)
    constexpr int df = 6;
    const double t_var = double(df) / double(df - 2);           // = 6/4 = 1.5
    const double t_std = std::sqrt(t_var);

    static thread_local std::mt19937 rng{std::random_device{}()};
    std::student_t_distribution<double> t_dist(df);

    // z ~ t_ν, standaryzujemy do wariancji 1 i skalujemy do docelowej wariancji
    const double z_unitvar = t_dist(rng) / t_std;                // var ≈ 1
    const double exec_time = mu + std::sqrt(std::max(0.0, var)) * z_unitvar;

    // Czas nie może być ujemny
    return std::max(exec_time, 0.0);
}

} // namespace lem_dynamics_sim_

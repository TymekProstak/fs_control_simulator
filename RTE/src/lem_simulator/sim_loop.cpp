#include "sim_loop.hpp"
#include <iostream>   // <<< diagnostyka
#include <exception>  // <<< diagnostyka



namespace lem_dynamics_sim_ {

    static inline int clamp_phase(int period, int phase)
    {
        if (period <= 1) return 0;
        phase %= period;
        if (phase < 0) phase += period;
        return phase;
    }
    
    static inline bool is_due(int step, int period, int phase)
    {
        if (period <= 0) return false;
        phase = clamp_phase(period, phase);
        return (step % period) == phase;
    }
    
    int Simulation_lem_ros_node::pick_phase_(int period)
    {
        if (period <= 1) return 0;
        std::uniform_int_distribution<int> dist(0, period - 1);
        return dist(phase_rng_);
    }
    


using json = nlohmann::json;

// ====== Konstruktor / inicjalizacja ======
Simulation_lem_ros_node::Simulation_lem_ros_node(ros::NodeHandle& nh,
                                                 const std::string& param_file,
                                                 const std::string& cones_file,
                                                 const std::string& log_file )
{
    // --- DIAG: wczytywanie parametrów ---
    std::cout << "[INIT] Opening param file: " << param_file << std::endl;
    {
        try {
            std::ifstream f(param_file);
            if (!f.is_open()) {
                std::cerr << "[INIT][FAIL] Cannot open param file." << std::endl;
                throw std::runtime_error("Nie mogę otworzyć pliku parametrów: " + param_file);
            }
            std::cout << "[INIT] Param file opened. Parsing JSON..." << std::endl;

            nlohmann::json J;
            f >> J;
            std::cout << "[INIT] JSON parsed. Building ParamBank..." << std::endl;

            P_ = build_param_bank(J);
            std::cout << "[INIT][OK] ParamBank built. Count=" << P_.size() << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "[INIT][FAIL] Parameter loading failed. what(): " << e.what() << std::endl;
            throw; // nie zmieniamy logiki — dalej rzucamy wyjątek
        }
    }

    // --- DIAG: reset stanu ---
    std::cout << "[INIT] Resetting simulation state..." << std::endl;
    state_.setZero();
    //state_.vx = 2.0;    


    // --- DIAG: wczytanie pachołków ---
    std::cout << "[INIT] Loading cones file: " << cones_file << std::endl;
    try {
        track_global_ = load_track_from_csv(cones_file);
        std::cout << "[INIT][OK] Cones loaded. cones=" << track_global_.cones.size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[INIT][FAIL] Cones load failed. what(): " << e.what() << std::endl;
        throw;
    }

    

    // 2) PID / sterowanie – reset
    std::cout << "[INIT] Resetting control/PID state..." << std::endl;
    pid_prev_I_ = 0.0;
    pid_prev_error_ = 0.0;
    target_wheel_speed_ = 0.0;
    target_wheel_speed_request = 0.0;
    pid_speed_out_ = 0.0;
    pid_omega_actual_ = 0.0;
    steer_command_ = 0.0; 
    torque_mode_ = 0; 
    torque_command_fl_ = 0.0;
    torque_command_fr_ = 0.0;
    torque_command_rl_ = 0.0;
    torque_command_rr_ = 0.0;
    torque_command_to_invert_fl_ = 0.0;
    torque_command_to_invert_fr_ = 0.0;
    torque_command_to_invert_rl_ = 0.0;
    torque_command_to_invert_rr_ = 0.0;

    // 3) Interwały krokowe (po wczytaniu P_)
    std::cout << "[INIT] Computing step intervals from parameters..." << std::endl;
    try {
        compute_step_intervals_from_params_();
        std::cout << "[INIT][OK] Step intervals computed." << std::endl;

        // ===== phases: rozstrzel czujniki/aktuatory w obrębie ich okresu =====
        phase_camera_shoot_          = pick_phase_(step_of_camera_shoot_);
        phase_wheel_encoder_reading_ = pick_phase_(step_of_wheel_encoder_reading_);
        phase_ins_reading_           = pick_phase_(step_of_ins_reading_);
        phase_torque_apply_          = pick_phase_(step_of_torque_input_application_);
        phase_steer_apply_           = pick_phase_(step_of_steer_input_application_);
        phase_pid_update_            = pick_phase_(step_number_pid_update_period_);

        std::cout << "[PHASES] cam="    << phase_camera_shoot_
                << " enc="            << phase_wheel_encoder_reading_
                << " ins="            << phase_ins_reading_
                << " torque_apply="   << phase_torque_apply_
                << " steer_apply="    << phase_steer_apply_
                << " pid="            << phase_pid_update_
          << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[INIT][FAIL] compute_step_intervals_from_params_ failed. what(): " << e.what() << std::endl;
        throw;
    }

    // 4) ROS I/O 
    std::cout << "[INIT] Initializing ROS I/O (subscribers/publishers)..." << std::endl;
    try {
        sub_control_ = nh.subscribe<dv_interfaces::Control>(
            "/dv_board/control", 1, &Simulation_lem_ros_node::dv_control_callback, this, ros::TransportHints().tcpNoDelay());

        pub_ins_   = nh.advertise<nav_msgs::Odometry>("/ins/pose", 1);
        pub_cones_ = nh.advertise<dv_interfaces::Cones>("/dv_cone_detector/cones", 1);
        pub_markers_cones_gt_  = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_gt", 1,true);
        pub_markers_cones_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_vis", 1);
        pub_log_full_ = nh.advertise<dv_interfaces::full_state>("/debug/full_log_info", 1);
        pub_marker_bolid_ = nh.advertise<visualization_msgs::Marker>("/viz/bolide_model", 1);
        std::cout << "[INIT][OK] ROS I/O ready." << std::endl;
        
        auto check_pub = [&](const char* name, const ros::Publisher& p){
            if (!p) {
                std::cerr << "[INIT][FAIL] Publisher invalid right after advertise(): " << name << std::endl;
                ROS_ERROR("Publisher invalid right after advertise(): %s", name);
            }
        };
        check_pub("pub_ins_",               pub_ins_);
        check_pub("pub_cones_",             pub_cones_);
        check_pub("pub_markers_cones_gt_",  pub_markers_cones_gt_);
        check_pub("pub_markers_cones_vis_", pub_markers_cones_vis_);

    } catch (const std::exception& e) {
        std::cerr << "[INIT][FAIL] ROS I/O init failed. what(): " << e.what() << std::endl;
        throw;
    }

    // 5) publikacja conów z toru (ground truth) o wiecznym life-time
    std::cout << "[INIT] Publishing GT cones markers..." << std::endl;
    try {
        publish_cones_gt_markers_();
        std::cout << "[INIT][OK] GT cones published." << std::endl;
    } catch (const std::exception& e) {
        // nie przerywamy — to tylko markery
        std::cerr << "[INIT][WARN] publish_cones_gt_markers_ failed. what(): " << e.what() << std::endl;
    }

    // 6) Logowanie - domyślnie wyłączone
    std::cout << "[INIT] Logging setup..." << std::endl;
    if (!log_file.empty()) {
        log_file_.open(log_file, std::ios::out);
        if (log_file_.is_open()) {
            logging_enabled_ = true;
            log_file_ << "time,x,y,yaw,vx,vy,yaw_rate,torque_fl,torque_fr,torque_rl,torque_rr,steer,"
                         "omega_fl,omega_fr,omega_rl,omega_rr,torque_request_fl,torque_request_fr,torque_request_rl,torque_request_rr,ax,ay\n";
            start_logging_thread_();
            std::cout << "[INIT][OK] Logging enabled -> " << log_file << std::endl;
        } else {
            std::cerr << "[INIT][WARN] Cannot open log file: " << log_file << std::endl;
            ROS_WARN("Nie udało się otworzyć pliku logowania: %s", log_file.c_str());
        }
    } else {
        std::cout << "[INIT] Logging disabled (empty path)." << std::endl;
    }

    // 7) Kolejka kamery pusta na start
    camera_queue_.clear();
    std::cout << "[INIT][DONE] Node constructed successfully." << std::endl;

    ROS_WARN_STREAM("Init yaw=" << state_.yaw << " vy=" << state_.vy);

}

// ====== Destruktor ======
Simulation_lem_ros_node::~Simulation_lem_ros_node() {
    // diagnostyka zamykania
    std::cout << "[DTOR] Shutting down logging thread..." << std::endl;
    stop_logging_thread_();
    if (log_file_.is_open()) {
        std::cout << "[DTOR] Flushing/closing log file..." << std::endl;
        log_file_.flush();
        log_file_.close();
    }
    std::cout << "[DTOR][DONE]" << std::endl;
}

// ====== Interfejs publiczny ======
void Simulation_lem_ros_node::step() {
    // minimalna diagnostyka kroku (bez floodu)
    // jeśli chcesz, możesz dodać licznik co N kroków

    // jako że symulacja tylko do testów kontroli to opoźnienie i szum sensorów ustawione w configu na zero
    // nie ma robienie strzelania zdjęc kamerą i kolejkowania
    try {
        // 1) Zastosuj wejścia z opóźnieniem
        apply_delayed_inputs_if_due_();

        // 2) Odczyty czujników wg harmonogramu
        read_wheel_encoder_if_due_();
        read_ins_if_due_();

        // 3) Aktualizacja PID
        if (torque_mode_ == 1)
            update_pid_if_due_(); // tryb speed
        
        // 5) Krok fizyki (Euler)
        {
            // In SPEED mode: PID returns a total torque request -> split equally.
            // In TORQUE mode: apply per-wheel torques from last command.
            Input(torque_command_rr_, torque_command_rl_, torque_command_fr_, torque_command_fl_, steer_command_);
            
            euler_sim_timestep(state_, in, P_);
        }

        if (logging_enabled_ && step_number_ % 10 == 0)
            log_state_();

        ++step_number_;

        pub_full_state_();
        publish_bolid_marker_();

    } catch (const std::exception& e) {
        std::cerr << "[STEP][FAIL] Exception in simulation step. what(): " << e.what() << std::endl;
        throw;
    }
}

State Simulation_lem_ros_node::get_state() const { return state_; }
ParamBank Simulation_lem_ros_node::get_parameters() const { return P_; }
int Simulation_lem_ros_node::get_step_number() const { return step_number_; }

// ====== ROS callback ======
void Simulation_lem_ros_node::dv_control_callback(const dv_interfaces::Control::ConstPtr& msg)
{
    DV_control_input u;
    u.torque_mode = static_cast<int>(msg->move_type);
    u.steer       = static_cast<double>(msg->steeringAngle_rad);

    if (u.torque_mode == 1) {
        // SPEED mode: use legacy movement field as target speed [km/h]
        u.speed_request_ms = static_cast<double>(msg->movement);
        // keep torques unchanged (0)
        u.torque_fl = u.torque_fr = u.torque_rl = u.torque_rr = 0.0;
    } else {
        // TORQUE mode: per-wheel torque requests in [%]
        u.speed_request_ms = 0.0;
        u.torque_fl = static_cast<double>(msg->torque_fl);
        u.torque_fr = static_cast<double>(msg->torque_fr);
        u.torque_rl = static_cast<double>(msg->torque_rl);
        u.torque_rr = static_cast<double>(msg->torque_rr);
    }

    last_input_requested_ = u;
}

// ====== Pomocnicze ======
void Simulation_lem_ros_node::compute_step_intervals_from_params_() {
    // diagnostyka bez zmian logiki (tylko odczyt wartości i print)
    const double dt = P_.get("simulation_time_step");
    std::cout << "[INTERVALS] dt=" << dt << std::endl;

    // UWAGA: poniższe P_.get(...) mogą rzucić, ale to zachowanie oryginalne
    step_of_camera_shoot_          = std::max(1, (int)std::round(1.0 / P_.get("frames_per_second") / dt));
    step_of_wheel_encoder_reading_ = std::max(1, (int)std::round(P_.get("wheel_encoder_reading_time_step") / dt));
    step_of_ins_reading_           = std::max(1, (int)std::round(1.0 / P_.get("ins_frequancy") / dt));
    step_of_torque_input_application_ = std::max(1, (int)std::round(P_.get("torque_input_delay") / dt));
    step_of_steer_input_application_  = std::max(1, (int)std::round(P_.get("steer_input_delay") / dt));
    step_number_pid_update_period_    = std::max(1, (int)std::round(P_.get("pid_time_step") / dt));

    std::cout << "[INTERVALS] cam="    << step_of_camera_shoot_
              << " enc="               << step_of_wheel_encoder_reading_
              << " ins="               << step_of_ins_reading_
              << " torque_apply="      << step_of_torque_input_application_
              << " steer_apply="       << step_of_steer_input_application_
              << " pid_period="        << step_number_pid_update_period_
              << std::endl;
}

void Simulation_lem_ros_node::apply_delayed_inputs_if_due_()
{
    if (is_due(step_number_, step_of_steer_input_application_, phase_steer_apply_)) {
        steer_command_ = last_input_requested_.steer;
    }

    if (is_due(step_number_, step_of_torque_input_application_, phase_torque_apply_)) {

        torque_mode_ = int(last_input_requested_.torque_mode);
        // jeśli sterotwanie momentem to na podajemy do symulacji jako akutalne momenty z ostatniego polecenia
        if (torque_mode_ == 0) {
            // TORQUE mode: scale each wheel [%] -> Nm
            const double scale = P_.get("max_torque") / 100.0;
            torque_command_fl_ = last_input_requested_.torque_fl * scale;
            torque_command_fr_ = last_input_requested_.torque_fr * scale;
            torque_command_rl_ = last_input_requested_.torque_rl * scale;
            torque_command_rr_ = last_input_requested_.torque_rr * scale;
        // jak nie to ustawiamy target speed w PID
        } else {
            // SPEED mode: update target speed request
            target_wheel_speed_ = last_input_requested_.speed_request_ms;
        }
    }
}

void Simulation_lem_ros_node::read_wheel_encoder_if_due_()
{
    if (step_of_wheel_encoder_reading_ <= 0) return;
    if (!is_due(step_number_, step_of_wheel_encoder_reading_, phase_wheel_encoder_reading_)) return;

    // 4x4: uśrednij 4 enkodery
    pid_omega_actual_ = 0.25 * (state_.omega_rr + state_.omega_rl + state_.omega_fr + state_.omega_fl);
}


void Simulation_lem_ros_node::read_ins_if_due_()
{
    if (step_of_ins_reading_ <= 0) return;
    if (!is_due(step_number_, step_of_ins_reading_, phase_ins_reading_)) return;

    const double f_ins = P_.get("ins_frequancy");
    const double Ts    = 1.0 / std::max(1e-6, f_ins);

    (void)Ts; // żeby nie krzyczało jeśli Ts chwilowo nieużyte

    ins_data_to_be_published_.x   = state_.x   + P_.get("pose_noise") * random_noise_generator_();
    ins_data_to_be_published_.y   = state_.y   + P_.get("pose_noise") * random_noise_generator_();
    ins_data_to_be_published_.yaw = state_.yaw + P_.get("orientation_noise") * random_noise_generator_();

    // prędkości w GLOBALU (world)
    double vx_global = state_.vx * std::cos(state_.yaw) - state_.vy * std::sin(state_.yaw);
    double vy_global = state_.vx * std::sin(state_.yaw) + state_.vy * std::cos(state_.yaw);

    ins_data_to_be_published_.vx = vx_global;
    ins_data_to_be_published_.vy = vy_global;

    ins_data_to_be_published_.yaw_rate =
        state_.yaw_rate + P_.get("rotation_noise") * random_noise_generator_();

    publish_ins_(ins_data_to_be_published_);
    last_ins_data_already_published_ = ins_data_to_be_published_;

    publish_bolid_tf_ins(ins_data_to_be_published_);
    publish_bolid_tf_true();
}

void Simulation_lem_ros_node::shoot_camera_or_enqueue_if_due_()
{
    if (step_of_camera_shoot_ <= 0) return;
    if (!is_due(step_number_, step_of_camera_shoot_, phase_camera_shoot_)) return;

    Track detection = shoot_a_frame(track_global_, P_, state_);
    const double dt = P_.get("simulation_time_step");
    const double vision_exec_time = sample_vision_exec_time_();
    const int processing_steps = std::max(0, (int)std::round(vision_exec_time / dt));

    CameraTask task;
    task.ready_step = step_number_ + processing_steps;
    task.frame      = std::move(detection);

    if ((int)camera_queue_.size() >= 3) {
        camera_queue_.pop_front();
        timestamp_queue_.pop_front();
    }

    camera_queue_.push_back(std::move(task));
    timestamp_queue_.push_back(ros::Time::now());
}


void Simulation_lem_ros_node::publish_ready_camera_frames_from_queue_() {
    while (!camera_queue_.empty() && camera_queue_.front().ready_step <= step_number_) {
        const auto& task = camera_queue_.front();
        const auto& timestamp = timestamp_queue_.front();
        publish_cones_(task.frame, timestamp);
        publish_cones_vision_markers_(task.frame, timestamp);
        camera_queue_.pop_front();
        timestamp_queue_.pop_front();
    }
}
void Simulation_lem_ros_node::update_pid_if_due_()
{
    if (torque_mode_ == 0) return;
    if (step_number_pid_update_period_ <= 0) return;
    if (!is_due(step_number_, step_number_pid_update_period_, phase_pid_update_)) return;

    double error = target_wheel_speed_ - pid_omega_actual_ * P_.get("R");

    pid_prev_I_ += (error + pid_prev_error_) / 2.0 * P_.get("pid_time_step");

    double u = P_.get("pid_p") * error +
               P_.get("pid_i") * pid_prev_I_ +
               P_.get("pid_d") * (error - pid_prev_error_) / P_.get("pid_time_step");

    pid_prev_error_ = error;

    u = std::clamp(u, P_.get("pid_min"), P_.get("pid_max"))
        * P_.get("max_torque") / P_.get("pid_scale");

    torque_command_to_invert_fl_ = u/4.0;
    torque_command_to_invert_fr_ = u/4.0;
    torque_command_to_invert_rl_ = u/4.0;
    torque_command_to_invert_rr_ = u/4.0;
}


double Simulation_lem_ros_node::random_noise_generator_() const {
    static thread_local std::mt19937 rng{std::random_device{}()};
    static thread_local std::normal_distribution<double> N01(0.0, 1.0);
    return N01(rng);
}

void Simulation_lem_ros_node::publish_ins_(const INS_data& ins){
    nav_msgs::Odometry odom_msg{};
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id  = "bolide_CoG";
    odom_msg.pose.pose.position.x = ins.x;
    odom_msg.pose.pose.position.y = ins.y;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, ins.yaw);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x  = ins.vx;
    odom_msg.twist.twist.linear.y  = ins.vy;
    odom_msg.twist.twist.angular.z = ins.yaw_rate;

    pub_ins_.publish(odom_msg);
}

void Simulation_lem_ros_node::publish_cones_(const Track& cones, ros::Time timestamp){
    if (!pub_cones_) {
        std::cerr << "[PUB][ERROR] pub_cones_ invalid; NOT publishing cones." << std::endl;
        ROS_ERROR("pub_cones_ invalid; not publishing cones");
        return;
    }
   
   
    dv_interfaces::Cones cones_msg;
    cones_msg.header.stamp = timestamp;
    cones_msg.header.frame_id = "camera_base";
    for (const auto& cone : cones.cones) {
        dv_interfaces::Cone cone_msg;
        cone_msg.confidence = 1.0;
        cone_msg.x = static_cast<float>(cone.x);
        cone_msg.y = static_cast<float>(cone.y);
        cone_msg.z = static_cast<float>(cone.z);
        cone_msg.distance_uncertainty = 0.0f;
        cone_msg.class_name = cone.color;
        cones_msg.cones.push_back(cone_msg);
    }
    pub_cones_.publish(cones_msg);
}

double Simulation_lem_ros_node::sample_vision_exec_time_() const {
    const double mu  = P_.get("mean_time_of_vision_execuction");
    const double var = P_.get("var_of_vision_time_execution");

    static thread_local std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<double> normal(mu, std::sqrt(std::max(0.0, var)));

    const double exec_time = normal(rng);
    return std::max(exec_time, 0.0);
}


void Simulation_lem_ros_node::publish_cones_vision_markers_(
    const Track& det, const ros::Time& acquisition_stamp)
{
    if (!pub_markers_cones_vis_) {
        ROS_ERROR("pub_markers_cones_vis_ invalid; not publishing vision markers");
        return;
    }

    visualization_msgs::MarkerArray arr;

    // ======================================================
    // 1) Usuń poprzednie markery wizji (TYLKO SWOJE)
    // ======================================================
    for (int id = 200; id < 200 + last_frame_size_; id++)
    {
        visualization_msgs::Marker del;
        del.header.frame_id = "camera_base";
        del.header.stamp    = acquisition_stamp;
        del.ns              = "cones_vis";
        del.id              = id;
        del.action          = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(del);
    }

    // ======================================================
    // 2) Dodaj nowe markery
    // ======================================================
    const double fps = std::max(1e-3, P_.get("frames_per_second"));
    const ros::Duration lifetime(5.0 / fps);

    int id = 200;
    for (const auto& c : det.cones)
    {
        std_msgs::ColorRGBA col = color_from_class_vision(c.color, 0.7f);

        visualization_msgs::Marker m;
        m.header.frame_id = "camera_base";
        m.header.stamp    = ros::Time::now();
        m.ns              = "cones_vis";
        m.id              = id++;
        m.type            = visualization_msgs::Marker::CUBE;
        m.action          = visualization_msgs::Marker::ADD;

        m.pose.position.x = c.x;
        m.pose.position.y = c.y;
        m.pose.position.z = c.z + 0.15;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.30;
        m.scale.y = 0.30;
        m.scale.z = 0.30;

        m.color    = col;
        m.lifetime = lifetime;
        m.frame_locked = false;

        arr.markers.push_back(std::move(m));
    }

    // ======================================================
    // 3) Zapisz ile markerów było w aktualnej ramce
    // ======================================================
    last_frame_size_ = det.cones.size();

    pub_markers_cones_vis_.publish(arr);
}


void Simulation_lem_ros_node::publish_bolid_tf_true() {
    geometry_msgs::TransformStamped tf_true;
    tf_true.header.stamp = ros::Time::now();
    tf_true.header.frame_id = "map";
    tf_true.child_frame_id  = "bolide_true";
    tf_true.transform.translation.x = state_.x;
    tf_true.transform.translation.y = state_.y;
    tf_true.transform.translation.z = 0.0;
    tf2::Quaternion q1; q1.setRPY(0, 0, state_.yaw);
    tf_true.transform.rotation.x = q1.x();
    tf_true.transform.rotation.y = q1.y();
    tf_true.transform.rotation.z = q1.z();
    tf_true.transform.rotation.w = q1.w();
    tf_br_.sendTransform(tf_true);
}




void Simulation_lem_ros_node::publish_bolid_tf_ins(const INS_data& ins){
    
    if (!pub_ins_) {
        std::cerr << "[PUB][ERROR] pub_ins_ invalid; NOT publishing INS." << std::endl;
        ROS_ERROR("pub_ins_ invalid; not publishing INS");
        return;
    }
    
    
    
    geometry_msgs::TransformStamped tf_ins;
    tf_ins.header.stamp = ros::Time::now();
    tf_ins.header.frame_id = "map";
    tf_ins.child_frame_id  = "bolide_CoG";
    tf_ins.transform.translation.x = ins.x;
    tf_ins.transform.translation.y = ins.y;
    tf_ins.transform.translation.z = 0.0;
    tf2::Quaternion q2; q2.setRPY(0, 0, ins.yaw);
    tf_ins.transform.rotation.x = q2.x();
    tf_ins.transform.rotation.y = q2.y();
    tf_ins.transform.rotation.z = q2.z();
    tf_ins.transform.rotation.w = q2.w();
    tf_br_.sendTransform(tf_ins);
}

// ====== Logger równoległy ======
void Simulation_lem_ros_node::start_logging_thread_() {
    if (!logging_enabled_) return;
    logging_thread_running_ = true;
    log_file_.rdbuf()->pubsetbuf(nullptr, 1 << 20);
    log_thread_ = std::thread([this]() {
        std::string batch;
        batch.reserve(1 << 18);
        while (logging_thread_running_) {
            {
                std::unique_lock<std::mutex> lock(log_mutex_);
                log_cv_.wait_for(lock, std::chrono::milliseconds(50),
                                 [this]() { return !log_queue_.empty() || !logging_thread_running_; });
                while (!log_queue_.empty()) {
                    batch += std::move(log_queue_.front());
                    log_queue_.pop();
                }
            }
            if (!batch.empty()) {
                log_file_ << batch;
                batch.clear();
            }
        }
        std::lock_guard<std::mutex> lock(log_mutex_);
        while (!log_queue_.empty()) {
            log_file_ << std::move(log_queue_.front());
            log_queue_.pop();
        }
        log_file_.flush();
    });
}

void Simulation_lem_ros_node::stop_logging_thread_() {
    if (!logging_enabled_) return;
    logging_thread_running_ = false;
    log_cv_.notify_all();
    if (log_thread_.joinable()) log_thread_.join();
}

void Simulation_lem_ros_node::log_state_() {
    std::ostringstream oss;
    oss.precision(5);
    oss.setf(std::ios::fixed);
    const float t = static_cast<float>(step_number_ * P_.get("simulation_time_step"));

    // log the per-wheel torque request that is applied by physics
    
    double rq_fl = torque_command_fl_;
     double   rq_fr = torque_command_fr_;
     double   rq_rl = torque_command_rl_;
     double    rq_rr = torque_command_rr_;
    

    oss << t << ","
        << static_cast<float>(state_.x) << ","
        << static_cast<float>(state_.y) << ","
        << static_cast<float>(state_.yaw) << ","
        << static_cast<float>(state_.vx) << ","
        << static_cast<float>(state_.vy) << ","
        << static_cast<float>(state_.yaw_rate) << ","
        << static_cast<float>(state_.torque_fl) << ","
        << static_cast<float>(state_.torque_fr) << ","
        << static_cast<float>(state_.torque_rl) << ","
        << static_cast<float>(state_.torque_rr) << ","
        << static_cast<float>(state_.rack_angle * M_PI / 180.0f) << ","
        << static_cast<float>(state_.omega_fl) << ","
        << static_cast<float>(state_.omega_fr) << ","
        << static_cast<float>(state_.omega_rl) << ","
        << static_cast<float>(state_.omega_rr) << ","
        << static_cast<float>(rq_fl) << ","
        << static_cast<float>(rq_fr) << ","
        << static_cast<float>(rq_rl) << ","
        << static_cast<float>(rq_rr) << ","
        << static_cast<float>(steer_command_ * M_PI / 180.0f) << ","
        << static_cast<float>(state_.prev_ay / 9.81f) << ","
        << static_cast<float>(state_.prev_ax / 9.81f) << "\n";
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_queue_.push(oss.str());
    }
    log_cv_.notify_one();
}

void Simulation_lem_ros_node::publish_cones_gt_markers_()
{
    visualization_msgs::MarkerArray arr;

    if (!pub_markers_cones_gt_) {
        std::cerr << "[PUB][ERROR] pub_markers_cones_gt_ invalid; NOT publishing GT markers." << std::endl;
        ROS_ERROR("pub_markers_cones_gt_ invalid; not publishing GT markers");
        return;
    }

    // (opcjonalnie) wyczyść poprzednie markery od tego publishera
    {
        visualization_msgs::Marker del;
        del.header.frame_id = "map";
        del.header.stamp    = ros::Time::now();
        del.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(del);
    }

    // lifetime = 0 → wieczny
    const ros::Duration kForever(0.0);

    int id = 300;
    for (const auto& c : track_global_.cones)
    {
        // kolor wg klasy (yellow/blue/orange/…)
        std_msgs::ColorRGBA col = color_from_class_gt(c.color, 0.95f);

        // bazowy “stożek” jako cylinder; funkcja ustawia ns="cones",
        // za chwilę nadpiszemy na "cones_gt" i frame na "map"
        visualization_msgs::Marker m = make_cone_marker(
            id++, /*frame*/ "map", c.x, c.y, c.z, col, kForever
        );

        // doprecyzowanie nagłówka/namespacu dla GT
        m.header.frame_id = "map";
        m.header.stamp    = ros::Time::now();
        m.ns = "cones_gt";                  // osobna przestrzeń nazw dla GT
        m.action = visualization_msgs::Marker::ADD;

        // (opcjonalnie) możesz różnić GT od wizji np. większą przezroczystością:
        // m.color.a = 0.7f;

        arr.markers.push_back(std::move(m));
    }

    pub_markers_cones_gt_.publish(arr);
}

void Simulation_lem_ros_node::pub_full_state_(){
    dv_interfaces::full_state msg;

    // build per-wheel input for log_info_full
    Input in;
    if (torque_mode_ == 1) {
        const double tq_total = torque_command_to_invert_;
        const double tq_w = 0.25 * tq_total;
        in = Input(tq_w, tq_w, tq_w, tq_w, steer_command_);
    } else {
        in = Input(torque_command_rr_, torque_command_rl_, torque_command_fr_, torque_command_fl_, steer_command_);
    }

    Log_Info_full info = log_info_full(state_, in, P_, step_number_);

    msg.time = info.time;
    msg.step_number = step_number_;

    msg.x = info.x;
    msg.y = info.y;
    msg.yaw = info.yaw;
    msg.yaw_rate = info.yaw_rate;
    msg.vx = info.vx;

    msg.vy = info.vy;
    msg.ax = info.ax;
    msg.ay = info.ay;

    // ===== 4x4: w dv_interfaces/full_state nadal mogą być stare pola.
    // Uzupełniamy nowymi jeśli istnieją, a stare (torque/left/right) wystawiamy jako sumy.

    // Sumaryczny torque (kompatybilność)
    msg.torque = info.torque_fl + info.torque_fr + info.torque_rl + info.torque_rr;

    // Jeżeli msg ma nadal torque_left/torque_right (legacy), przypnij przód+tył na stronę
    msg.torque_left  = info.torque_fl + info.torque_rl;
    msg.torque_right = info.torque_fr + info.torque_rr;

    // Legacy omega fields jeśli tylko rl/rr
    msg.omega_rl = info.omega_rl;
    msg.omega_rr = info.omega_rr;

    msg.rack_angle = info.rack_angle;
    msg.delta_left = info.delta_left;
    msg.delta_rigth = info.delta_rigth;
    msg.rack_angle_request = info.rack_angle_request;

    msg.fx_fl = info.fx_fl;
    msg.fx_fr = info.fx_fr;
    msg.fx_rl = info.fx_rl;
    msg.fx_rr = info.fx_rr;

    msg.fy_fl = info.fy_fl;
    msg.fy_fr = info.fy_fr;
    msg.fy_rl = info.fy_rl;
    msg.fy_rr = info.fy_rr;

    msg.fz_fl = info.fz_fl;
    msg.fz_fr = info.fz_fr;
    msg.fz_rl = info.fz_rl;
    msg.fz_rr = info.fz_rr;

    msg.slip_angle_fl = info.slip_angle_fl;
    msg.slip_angle_fr = info.slip_angle_fr;
    msg.slip_angle_rl = info.slip_angle_rl;
    msg.slip_angle_rr = info.slip_angle_rr;
    msg.slip_angle_body = info.slip_angle_body;

    msg.kappa_fl = info.kappa_fl;
    msg.kappa_fr = info.kappa_fr;
    msg.kappa_rl = info.kappa_rl;
    msg.kappa_rr = info.kappa_rr;

    msg.total_drag = info.total_drag;
    msg.total_downforce = info.total_downforce;
    msg.Power_total = info.Power_total;

    msg.step_dt = P_.get("simulation_time_step");

    pub_log_full_.publish(msg);
}

void Simulation_lem_ros_node::publish_bolid_marker_()
{
    if (!pub_marker_bolid_) {
        ROS_ERROR("pub_marker_bolid_ invalid; not publishing bolid marker");
        return;
    }

    visualization_msgs::Marker car;
    car.header.frame_id = "bolide_true";        // auto porusza się z TF pojazdu
    car.header.stamp    = ros::Time::now();
    car.ns   = "bolide";
    car.id   = 0;
    car.type = visualization_msgs::Marker::CUBE;
    car.action = visualization_msgs::Marker::ADD;

    // --- rozmiar pojazdu FS (około) ---
    car.scale.x = 2.8;    // długość (m)
    car.scale.y = 1.5;    // szerokość (m)
    car.scale.z = 1.5;    // wysokość (m)

    // --- pozycja ---
    car.pose.position.x = 0.0;      // środek ciężkości = TF origin
    car.pose.position.y = 0.0;
    car.pose.position.z = 0.75;     // połowa wysokości, żeby stał na ziemi
    car.pose.orientation.w = 1.0;

    car.color.r = 0.0f;
    car.color.g = 0.9f;
    car.color.b = 0.2f;
    car.color.a = 1.0f;

    // --- lifetime krótkie, auto się odświeża ---
    car.lifetime = ros::Duration(0.1);

    pub_marker_bolid_.publish(car);
}

}// namespace lem_dynamics_sim_




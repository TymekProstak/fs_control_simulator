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
    publish_cones_gt_markers_(); // publikacja conów z toru (ground truth) o wiecznym life-time

    // 2) PID / sterowanie – reset
    pid_prev_I_ = 0.0;
    pid_prev_error_ = 0.0;
    target_wheel_speed_ = 0.0;
    pid_speed_out_ = 0.0;
    pid_omega_actual_ = 0.0;
    torque_command_ = 0.0; 
    torque_command_to_invert_ = 0.0;
    steer_command_ = 0.0;
    torque_mode_ = 0; 

    // 3) Interwały krokowe (po wczytaniu P_)
    compute_step_intervals_from_params_();

    // 4) ROS I/O 
    sub_control_ = nh.subscribe<dv::interfaces::Control>(
        "/dv_board/control", 1, &Simulation_lem_ros_node::dv_control_callback, this, ros::TransportHints().tcpNoDelay());

    pub_ins_   = nh.advertise<nav_msgs::Odometry>("/ins/pose", 1);
    pub_cones_ = nh.advertise<dv_interfaces::Cones>("/dv_cone_detector/cones", 1);
    pub_markers_cones_gt_  = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_gt", 1);
    pub_markers_cones_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_vis", 1);

    // 5) Logowanie - domyślnie wyłączone
    if (!log_file.empty()) {
        log_file_.open(log_file, std::ios::out);
        if (log_file_.is_open()) {
            logging_enabled_ = true;
            log_file_ << "time,x,y,yaw,vx,vy,yaw_rate,torque,steer,"
                         "omega_rr,omega_rl,torque_request,steer_request,ax,ay\n";
            start_logging_thread_();
        } else {
            ROS_WARN("Nie udało się otworzyć pliku logowania: %s", log_file.c_str());
        }
    }

    // 6) Kolejka kamery pusta na start
    camera_queue_.clear();
}

// ====== Destruktor ======
Simulation_lem_ros_node::~Simulation_lem_ros_node() {
    stop_logging_thread_();
    if (log_file_.is_open()) {
        log_file_.flush();
        log_file_.close();
    }
}

// ====== Interfejs publiczny ======
void Simulation_lem_ros_node::step() {
    // 1) Zastosuj wejścia z opóźnieniem
    apply_delayed_inputs_if_due_();

    // 2) Odczyty czujników wg harmonogramu
    read_wheel_encoder_if_due_();
    read_ins_if_due_();
    shoot_camera_or_enqueue_if_due_();

    // 3) Aktualizacja PID
    if (torque_mode_ == 1)
        update_pid_if_due_(); // tryb speed
    else
        torque_command_to_invert_ = torque_command_; // tryb torque

    // 4) Publikacje ramek, które „dojrzały”
    publish_ready_camera_frames_from_queue_();

    // 5) Krok fizyki (Euler)
    euler_sim_timestep(state_, Input(torque_command_to_invert_, steer_command_), P_);

    // 6) Logowanie co 10 kroków
    if (logging_enabled_ && step_number_ % 10 == 0)
        log_state_();

    // 7) Inkrement kroku
    ++step_number_;
}

State Simulation_lem_ros_node::get_state() const { return state_; }
ParamBank Simulation_lem_ros_node::get_parameters() const { return P_; }
int Simulation_lem_ros_node::get_step_number() const { return step_number_; }

// ====== ROS callback ======
void Simulation_lem_ros_node::dv_control_callback(const dv_interfaces::Control &msg) {
    DV_control_input u;
    u.torque = static_cast<double>(msg.movement);
    u.steer  = static_cast<double>(msg.steeringAngle_rad);
    u.torque_mode = static_cast<int>(msg.move_type);
    last_input_requested_ = u;
}

// ====== Pomocnicze ======
void Simulation_lem_ros_node::compute_step_intervals_from_params_() {
    const double dt = P_.get("simulation_time_step");

    step_of_camera_shoot_          = std::max(1, (int)std::round(1.0 / P_.get("frames_per_second") / dt));
    step_of_wheel_encoder_reading_ = std::max(1, (int)std::round(P_.get("wheel_encoder_reading_time_step") / dt));
    step_of_ins_reading_           = std::max(1, (int)std::round(1.0 / P_.get("ins_frequancy") / dt));
    step_of_torque_input_application_ = std::max(1, (int)std::round(P_.get("torque_input_delay") / dt));
    step_of_steer_input_application_  = std::max(1, (int)std::round(P_.get("steer_input_delay") / dt));
    step_number_pid_update_period_    = std::max(1, (int)std::round(P_.get("pid_time_step") / dt));
}

void Simulation_lem_ros_node::apply_delayed_inputs_if_due_() {
    if (step_number_to_apply_steer_input_ > 0 && step_number_ % step_of_steer_input_application_ == 0) {
        steer_command_ = last_input_requested_.steer;
    }
    if (step_number_to_apply_torque_input_ > 0 && step_number_ % step_of_torque_input_application_ == 0) {
        torque_command_ = double(last_input_requested_.torque);
        torque_mode_    = int(last_input_requested_.torque_mode);
    }
}

void Simulation_lem_ros_node::read_wheel_encoder_if_due_() {
    if (step_of_wheel_encoder_reading_ <= 0) return;
    if (step_number_ % step_of_wheel_encoder_reading_ != 0) return;
    pid_omega_actual_ = (state_.omega_rr + state_.omega_rl) * 0.5;
}

void Simulation_lem_ros_node::read_ins_if_due_() {
    if (step_of_ins_reading_ <= 0) return;
    if (step_number_ % step_of_ins_reading_ != 0) return;

    const double f_ins = P_.get("ins_frequancy");
    const double Ts    = 1.0 / std::max(1e-6, f_ins);

    ins_data_to_be_published_.x   = state_.x   + P_.get("ins_x_noise")       * random_noise_generator_();
    ins_data_to_be_published_.y   = state_.y   + P_.get("ins_y_noise")       * random_noise_generator_();
    ins_data_to_be_published_.yaw = state_.yaw + P_.get("ins_yaw_noise")     * random_noise_generator_();
    ins_data_to_be_published_.vx  = (ins_data_to_be_published_.x - last_ins_data_already_published_.x) / Ts;
    ins_data_to_be_published_.vy  = (ins_data_to_be_published_.y - last_ins_data_already_published_.y) / Ts;
    ins_data_to_be_published_.yaw_rate = state_.yaw_rate + P_.get("ins_yaw_rate_noise") * random_noise_generator_();

    publish_ins_(ins_data_to_be_published_);
    last_ins_data_already_published_ = ins_data_to_be_published_;

    publish_bolid_tf_ins(ins_data_to_be_published_);
    publish_bolid_tf_true();
}

void Simulation_lem_ros_node::shoot_camera_or_enqueue_if_due_() {
    if (step_of_camera_shoot_ <= 0) return;
    if (step_number_ % step_of_camera_shoot_ != 0) return;

    Track detection = get_cone_detections_in_camera_frame(state_, track_global_, P_);
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

void Simulation_lem_ros_node::update_pid_if_due_() {
    if (torque_mode_ == 0) return;
    if (step_number_pid_update_period_ <= 0) return;
    if (step_number_ % step_number_pid_update_period_ != 0) return;

    target_wheel_speed_ = torque_command_;
    double error = target_wheel_speed_ - pid_omega_actual_ * P_.get("R");
    pid_prev_I_ += (error + pid_prev_error_) / 2 * P_.get("pid_time_step");
    double u = P_.get("pid_p") * error +
               P_.get("pid_i") * pid_prev_I_ +
               P_.get("pid_d") * (error - pid_prev_error_) / P_.get("pid_time_step");
    pid_prev_error_ = error;
    torque_command_to_invert_ = u; 
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
    const double mu  = P_.get("vision.mean_time_of_vision_execuction");
    const double var = P_.get("vision.var_of_vision_time_execution");
    constexpr int df = 6;
    const double t_std = std::sqrt(double(df) / double(df - 2));
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::student_t_distribution<double> t_dist(df);
    const double z = t_dist(rng) / t_std;
    const double exec_time = mu + std::sqrt(std::max(0.0, var)) * z;
    return std::max(exec_time, 0.0);
}

void Simulation_lem_ros_node::publish_cones_vision_markers_(const Track& det, const ros::Time& acquisition_stamp) {
    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker del;
    del.header.frame_id = "camera_base";
    del.header.stamp    = ros::Time::now();
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    const ros::Duration T(std::max(0.05, 1.0 / std::max(P_.get("frames_per_second"), 1e-6)));
    int id = 0;
    for (const auto& c : det.cones) {
        auto col = color_from_class(c.color, 0.95f);
        visualization_msgs::Marker m = make_cone_marker(id++, "camera_base", c.x, c.y, c.z, col, T);
        m.header.stamp = acquisition_stamp;
        arr.markers.push_back(std::move(m));
    }
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
    oss << t << ","
        << static_cast<float>(state_.x) << ","
        << static_cast<float>(state_.y) << ","
        << static_cast<float>(state_.yaw) << ","
        << static_cast<float>(state_.vx) << ","
        << static_cast<float>(state_.vy) << ","
        << static_cast<float>(state_.yaw_rate) << ","
        << static_cast<float>(state_.torque) << ","
        << static_cast<float>(state_.rack_angle * M_PI / 180.0f) << ","
        << static_cast<float>(state_.omega_rr) << ","
        << static_cast<float>(state_.omega_rl) << ","
        << static_cast<float>(torque_command_) << ","
        << static_cast<float>(steer_command_ * M_PI / 180.0f) << ","
        << static_cast<float>(state_.ax / 9.81f) << ","
        << static_cast<float>(state_.ay / 9.81f) << "\n";
    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_queue_.push(oss.str());
    }
    log_cv_.notify_one();
}

} // namespace lem_dynamics_sim_

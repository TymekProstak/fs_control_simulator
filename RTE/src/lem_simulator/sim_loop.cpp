#include "sim_loop.hpp"

#include <iostream>
#include <exception>
#include <limits>
#include <iomanip>

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

// ============================================================================
//  CTOR
// ============================================================================
Simulation_lem_ros_node::Simulation_lem_ros_node(ros::NodeHandle& nh,
                                                 const std::string& param_file,
                                                 const std::string& track_file,
                                                 const std::string& log_file)
{
    // 1) Params
    {
        std::ifstream f(param_file);
        if (!f.is_open())
            throw std::runtime_error("Nie mogę otworzyć pliku parametrów: " + param_file);

        nlohmann::json J;
        f >> J;
        P_ = build_param_bank(J);
    }

    // 2) Reset stanu
    state_.setZero();

    // 3) Track
    track_global_ = load_track_from_csv(track_file);

    // 4) Reset sterowania / PID
    pid_prev_I_ = 0.0;
    pid_prev_error_ = 0.0;

    target_wheel_speed_ = 0.0;
    target_wheel_speed_request = 0.0;

    pid_speed_out_ = 0.0;
    pid_omega_actual_ = 0.0;

    torque_command_to_invert_fl_ = 0.0;
    torque_command_to_invert_fr_ = 0.0;
    torque_command_to_invert_rl_ = 0.0;
    torque_command_to_invert_rr_ = 0.0;

    torque_command_recived_by_dv_board_fl_ = 0.0;
    torque_command_recived_by_dv_board_fr_ = 0.0;
    torque_command_recived_by_dv_board_rl_ = 0.0;
    torque_command_recived_by_dv_board_rr_ = 0.0;

    steer_command_recived_by_dv_board = 0.0;
    steer_command_request = 0.0;

    torque_mode_ = 0;
    last_input_requested_ = DV_control_input{};

    // 5) Interwały
    compute_step_intervals_from_params_();

    // 6) Fazy (rozstrzelenie w obrębie okresu)
    phase_camera_shoot_          = pick_phase_(step_of_camera_shoot_);
    phase_wheel_encoder_reading_ = pick_phase_(step_of_wheel_encoder_reading_);
    phase_ins_reading_           = pick_phase_(step_of_ins_reading_);

    // READ / SEND
    phase_torque_apply_ = pick_phase_(step_of_control_input_read_);
    phase_steer_apply_  = pick_phase_(step_of_steer_input_sending_);
    phase_pid_update_   = pick_phase_(step_number_torque_input_sending_);
    phase_gps_speed_reading_    = pick_phase_(step_gps_speed_reading_);

    // 7) ROS I/O
    sub_control_ = nh.subscribe<dv_interfaces::Control>(
        "/dv_board/control", 1, &Simulation_lem_ros_node::dv_control_callback, this,
        ros::TransportHints().tcpNoDelay());

    pub_ins_   = nh.advertise<nav_msgs::Odometry>("/ins/pose", 1);
    pub_cones_ = nh.advertise<dv_interfaces::Cones>("/dv_cone_detector/cones", 1);

    pub_markers_cones_gt_  = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_gt", 1, true);
    pub_markers_cones_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_vis", 1);

    pub_log_full_     = nh.advertise<dv_interfaces::full_state>("/debug/full_log_info", 1);
    pub_marker_bolid_ = nh.advertise<visualization_msgs::Marker>("/viz/bolide_model", 1);

    // 8) publish GT cones (wieczne)
    publish_cones_gt_markers_();

    // 9) Logowanie
    if (!log_file.empty()) {
        log_file_.open(log_file, std::ios::out);
        if (log_file_.is_open()) {
            logging_enabled_ = true;
            log_file_ << "time,x,y,yaw,vx,vy,yaw_rate,"
                         "torque_fl,torque_fr,torque_rl,torque_rr,"
                         "steer,"
                         "omega_fl,omega_fr,omega_rl,omega_rr,"
                         "torque_req_fl,torque_req_fr,torque_req_rl,torque_req_rr,"
                         "ay_g,ax_g\n";
            start_logging_thread_();
        }
    }

    camera_queue_.clear();
    timestamp_queue_.clear();
}

Simulation_lem_ros_node::~Simulation_lem_ros_node()
{
    stop_logging_thread_();
    if (log_file_.is_open()) {
        log_file_.flush();
        log_file_.close();
    }
}

// ============================================================================
//  Public
// ============================================================================
void Simulation_lem_ros_node::step()
{
    // 1) READ/SEND pipeline (to jest jedyne miejsce gdzie zmieniam sterowanie)
    apply_delayed_inputs_if_due_();

    // 2) Sensory wg harmonogramu
    read_wheel_encoder_if_due_();
    read_ins_if_due_();

    // 3) (opcjonalnie) kamera — jak chcesz, możesz znowu włączyć
    // shoot_camera_or_enqueue_if_due_();
    // publish_ready_camera_frames_from_queue_();

    // 4) Fizyka: zawsze używam "tego co już wyszło na aktuatory"
    {
        Input in(
            torque_command_to_invert_rr_,
            torque_command_to_invert_rl_,
            torque_command_to_invert_fr_,
            torque_command_to_invert_fl_,
            steer_command_request
        );

        euler_sim_timestep(state_, in, P_);
    }

    if (logging_enabled_ && (step_number_ % 10 == 0))
        log_state_();

    ++step_number_;

    pub_full_state_();
    publish_bolid_marker_();
}

State Simulation_lem_ros_node::get_state() const { return state_; }
ParamBank Simulation_lem_ros_node::get_parameters() const { return P_; }
int Simulation_lem_ros_node::get_step_number() const { return step_number_; }

// ============================================================================
//  ROS callback: tylko buforuję (asynchronicznie), NIC nie aplikuję
// ============================================================================
void Simulation_lem_ros_node::dv_control_callback(const dv_interfaces::Control::ConstPtr& msg)
{
    DV_control_input u;
    u.torque_mode = static_cast<int>(msg->move_type);
    u.steer       = static_cast<double>(msg->steeringAngle_rad);

    if (u.torque_mode == 1) {
        u.speed_request_ms = static_cast<double>(msg->movement);
        u.torque_fl = u.torque_fr = u.torque_rl = u.torque_rr = 0.0;
    } else {
        u.speed_request_ms = 0.0;
        u.torque_fl = static_cast<double>(msg->torque_fl);
        u.torque_fr = static_cast<double>(msg->torque_fr);
        u.torque_rl = static_cast<double>(msg->torque_rl);
        u.torque_rr = static_cast<double>(msg->torque_rr);
    }

    last_input_requested_ = u;
}

// ============================================================================
//  Interwały: tylko SEND/READ mają znaczenie dla IO
// ============================================================================
void Simulation_lem_ros_node::compute_step_intervals_from_params_()
{
    const double dt = P_.get("simulation_time_step");

    step_of_camera_shoot_          = std::max(1, (int)std::round(1.0 / P_.get("frames_per_second") / dt));
    step_of_wheel_encoder_reading_ = std::max(1, (int)std::round(P_.get("wheel_encoder_reading_time_steep") / dt));
    step_of_ins_reading_           = std::max(1, (int)std::round(1.0 / P_.get("ins_frequancy") / dt));
    step_gps_speed_reading_        = std::max(1, (int)std::round(1.0 / P_.get("gps_speed_frequancy") / dt));

    // ===== READ/SEND =====
    step_of_control_input_read_       = std::max(1, (int)std::round(P_.get("control_to_dv_boad_read_time_step") / dt));
    step_of_steer_input_sending_      = std::max(1, (int)std::round(P_.get("dv_board_to_maxon_time_step") / dt));
    step_number_torque_input_sending_ = std::max(1, (int)std::round(P_.get("dv_board_tractive_system_time_step") / dt));
}

// ============================================================================
//  SEND/READ PIPELINE (w środku trzymam 3 eventy)
// ============================================================================
void Simulation_lem_ros_node::apply_delayed_inputs_if_due_()
{
    // ============================================================
    // 1) DV_BOARD_READ: board czyta z ROS w swoim ticku READ
    // ============================================================
    if (is_due(step_number_, step_of_control_input_read_, phase_torque_apply_)) {

        torque_mode_ = int(last_input_requested_.torque_mode);

        // steering: board latch (jeszcze nie wysyłam do aktuatora)
        steer_command_recived_by_dv_board = last_input_requested_.steer;

        if (torque_mode_ == 0) {
            // TORQUE mode: [%] -> [Nm] i latch na boardzie
            const double scale = P_.get("max_torque") / 100.0;

            torque_command_recived_by_dv_board_fl_ = last_input_requested_.torque_fl * scale;
            torque_command_recived_by_dv_board_fr_ = last_input_requested_.torque_fr * scale;
            torque_command_recived_by_dv_board_rl_ = last_input_requested_.torque_rl * scale;
            torque_command_recived_by_dv_board_rr_ = last_input_requested_.torque_rr * scale;
        } else {
            // SPEED mode: latch target speed
            target_wheel_speed_request = last_input_requested_.speed_request_ms;
            target_wheel_speed_ = target_wheel_speed_request;
        }
    }

    // ============================================================
    // 2) SEND_STEER: board wysyła steering do maxona
    // ============================================================
    if (is_due(step_number_, step_of_steer_input_sending_, phase_steer_apply_)) {
        steer_command_request = steer_command_recived_by_dv_board;
    }

    // ============================================================
    // 3) SEND_TORQUE: board wysyła torque do tractive system
    //    (PID liczę dokładnie na tym samym ticku co SEND)
    // ============================================================
    if (is_due(step_number_, step_number_torque_input_sending_, phase_pid_update_)) {

        if (torque_mode_ == 0) {
            // TORQUE mode: wysyłam to, co było zatrzaśnięte w READ
            torque_command_to_invert_fl_ = torque_command_recived_by_dv_board_fl_;
            torque_command_to_invert_fr_ = torque_command_recived_by_dv_board_fr_;
            torque_command_to_invert_rl_ = torque_command_recived_by_dv_board_rl_;
            torque_command_to_invert_rr_ = torque_command_recived_by_dv_board_rr_;
        } else {
            // SPEED mode: liczę PID i wysyłam jego wynik
            update_pid_if_due_(); // tu NIE ma już drugiego is_due()
        }
    }
}

// ============================================================================
//  Encoders / INS
// ============================================================================
void Simulation_lem_ros_node::read_wheel_encoder_if_due_()
{
    if (!is_due(step_number_, step_of_wheel_encoder_reading_, phase_wheel_encoder_reading_))
        return;

    pid_omega_actual_ = 0.25 * (state_.omega_rr + state_.omega_rl + state_.omega_fr + state_.omega_fl);
}

void Simulation_lem_ros_node::read_ins_if_due_()
{
    const bool due_ins = is_due(step_number_, step_of_ins_reading_, phase_ins_reading_);
    const bool due_gps = is_due(step_number_, step_gps_speed_reading_, phase_gps_speed_reading_);

    // nic do roboty w tym ticku
    if (!due_ins && !due_gps)
        return;

    if (due_gps) {
        const double vx_global = state_.vx * std::cos(state_.yaw) - state_.vy * std::sin(state_.yaw);
        const double vy_global = state_.vx * std::sin(state_.yaw) + state_.vy * std::cos(state_.yaw);

        ins_data_to_be_published_.vx = vx_global + P_.get("speed_gps_noise") * random_noise_generator_();
        ins_data_to_be_published_.vy = vy_global + P_.get("speed_gps_noise") * random_noise_generator_();

        last_ins_data_already_published_ = ins_data_to_be_published_;
    }

    if (due_ins) {
        ins_data_to_be_published_.x   = state_.x   + P_.get("pose_noise") * random_noise_generator_();
        ins_data_to_be_published_.y   = state_.y   + P_.get("pose_noise") * random_noise_generator_();
        ins_data_to_be_published_.yaw = state_.yaw + P_.get("orientation_noise") * random_noise_generator_();

        ins_data_to_be_published_.yaw_rate =
            state_.yaw_rate + P_.get("rotation_noise") * random_noise_generator_();

        publish_ins_(ins_data_to_be_published_);
        last_ins_data_already_published_ = ins_data_to_be_published_;

        publish_bolid_tf_ins(ins_data_to_be_published_);
        publish_bolid_tf_true();
    }

 
}

// ============================================================================
//  PID: bez własnego is_due() — wołam go tylko w ticku SEND_TORQUE
// ============================================================================
void Simulation_lem_ros_node::update_pid_if_due_()
{
    if (torque_mode_ == 0) return;

    double error = target_wheel_speed_ - pid_omega_actual_ * P_.get("R");

    pid_prev_I_ += (error + pid_prev_error_) / 2.0 * P_.get("pid_time_step");

    double u = P_.get("pid_p") * error +
               P_.get("pid_i") * pid_prev_I_ +
               P_.get("pid_d") * (error - pid_prev_error_) / P_.get("pid_time_step");

    pid_prev_error_ = error;

    u = std::clamp(u, P_.get("pid_min"), P_.get("pid_max"))
        * P_.get("max_torque") / P_.get("pid_scale");

    torque_command_to_invert_fl_ = u / 4.0;
    torque_command_to_invert_fr_ = u / 4.0;
    torque_command_to_invert_rl_ = u / 4.0;
    torque_command_to_invert_rr_ = u / 4.0;
}

// ============================================================================
//  Noise
// ============================================================================
double Simulation_lem_ros_node::random_noise_generator_() const
{
    static thread_local std::mt19937 rng{std::random_device{}()};
    static thread_local std::normal_distribution<double> N01(0.0, 1.0);
    return N01(rng);
}

// ============================================================================
//  Publish INS
// ============================================================================
void Simulation_lem_ros_node::publish_ins_(const INS_data& ins)
{
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

// ============================================================================
//  Publish cones
// ============================================================================
void Simulation_lem_ros_node::publish_cones_(const Track& cones, ros::Time timestamp)
{
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

// ============================================================================
//  Vision exec time
// ============================================================================
double Simulation_lem_ros_node::sample_vision_exec_time_() const
{
    const double mu  = P_.get("mean_time_of_vision_execuction");
    const double var = P_.get("var_of_vision_time_execution");

    static thread_local std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<double> normal(mu, std::sqrt(std::max(0.0, var)));

    const double exec_time = normal(rng);
    return std::max(exec_time, 0.0);
}

// ============================================================================
//  Vision markers
// ============================================================================
void Simulation_lem_ros_node::publish_cones_vision_markers_(
    const Track& det, const ros::Time& acquisition_stamp)
{
    visualization_msgs::MarkerArray arr;

    for (int id = 200; id < 200 + last_frame_size_; id++) {
        visualization_msgs::Marker del;
        del.header.frame_id = "camera_base";
        del.header.stamp    = acquisition_stamp;
        del.ns              = "cones_vis";
        del.id              = id;
        del.action          = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(del);
    }

    const double fps = std::max(1e-3, P_.get("frames_per_second"));
    const ros::Duration lifetime(5.0 / fps);

    int id = 200;
    for (const auto& c : det.cones) {
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

    last_frame_size_ = (int)det.cones.size();
    pub_markers_cones_vis_.publish(arr);
}

// ============================================================================
//  TF
// ============================================================================
void Simulation_lem_ros_node::publish_bolid_tf_true()
{
    geometry_msgs::TransformStamped tf_true;
    tf_true.header.stamp = ros::Time::now();
    tf_true.header.frame_id = "map";
    tf_true.child_frame_id  = "bolide_true";

    tf_true.transform.translation.x = state_.x;
    tf_true.transform.translation.y = state_.y;
    tf_true.transform.translation.z = 0.0;

    tf2::Quaternion q; q.setRPY(0, 0, state_.yaw);
    tf_true.transform.rotation.x = q.x();
    tf_true.transform.rotation.y = q.y();
    tf_true.transform.rotation.z = q.z();
    tf_true.transform.rotation.w = q.w();

    tf_br_.sendTransform(tf_true);
}

void Simulation_lem_ros_node::publish_bolid_tf_ins(const INS_data& ins)
{
    geometry_msgs::TransformStamped tf_ins;
    tf_ins.header.stamp = ros::Time::now();
    tf_ins.header.frame_id = "map";
    tf_ins.child_frame_id  = "bolide_CoG";

    tf_ins.transform.translation.x = ins.x;
    tf_ins.transform.translation.y = ins.y;
    tf_ins.transform.translation.z = 0.0;

    tf2::Quaternion q; q.setRPY(0, 0, ins.yaw);
    tf_ins.transform.rotation.x = q.x();
    tf_ins.transform.rotation.y = q.y();
    tf_ins.transform.rotation.z = q.z();
    tf_ins.transform.rotation.w = q.w();

    tf_br_.sendTransform(tf_ins);
}

// ============================================================================
//  Logger thread
// ============================================================================
void Simulation_lem_ros_node::start_logging_thread_()
{
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

void Simulation_lem_ros_node::stop_logging_thread_()
{
    if (!logging_enabled_) return;

    logging_thread_running_ = false;
    log_cv_.notify_all();
    if (log_thread_.joinable()) log_thread_.join();
}

void Simulation_lem_ros_node::log_state_()
{
    std::ostringstream oss;
    oss.precision(5);
    oss.setf(std::ios::fixed);

    const float t = static_cast<float>(step_number_ * P_.get("simulation_time_step"));

    // Ja loguję to, co realnie poszło "na inwertery"
    const double rq_fl = torque_command_to_invert_fl_;
    const double rq_fr = torque_command_to_invert_fr_;
    const double rq_rl = torque_command_to_invert_rl_;
    const double rq_rr = torque_command_to_invert_rr_;

    oss << t << ","
        << (float)state_.x << ","
        << (float)state_.y << ","
        << (float)state_.yaw << ","
        << (float)state_.vx << ","
        << (float)state_.vy << ","
        << (float)state_.yaw_rate << ","
        << (float)state_.torque_fl << ","
        << (float)state_.torque_fr << ","
        << (float)state_.torque_rl << ","
        << (float)state_.torque_rr << ","
        << (float)steer_command_request << ","
        << (float)state_.omega_fl << ","
        << (float)state_.omega_fr << ","
        << (float)state_.omega_rl << ","
        << (float)state_.omega_rr << ","
        << (float)rq_fl << ","
        << (float)rq_fr << ","
        << (float)rq_rl << ","
        << (float)rq_rr << ","
        << (float)(state_.prev_ay / 9.81f) << ","
        << (float)(state_.prev_ax / 9.81f) << "\n";

    {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_queue_.push(oss.str());
    }
    log_cv_.notify_one();
}

// ============================================================================
//  GT cones markers
// ============================================================================
void Simulation_lem_ros_node::publish_cones_gt_markers_()
{
    visualization_msgs::MarkerArray arr;

    visualization_msgs::Marker del;
    del.header.frame_id = "map";
    del.header.stamp    = ros::Time::now();
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    const ros::Duration kForever(0.0);

    int id = 300;
    for (const auto& c : track_global_.cones) {
        std_msgs::ColorRGBA col = color_from_class_gt(c.color, 0.95f);

        visualization_msgs::Marker m = make_cone_marker(
            id++, "map", c.x, c.y, c.z, col, kForever
        );

        m.header.frame_id = "map";
        m.header.stamp    = ros::Time::now();
        m.ns = "cones_gt";
        m.action = visualization_msgs::Marker::ADD;

        arr.markers.push_back(std::move(m));
    }

    pub_markers_cones_gt_.publish(arr);
}

// ============================================================================
//  full_state publisher
// ============================================================================
void Simulation_lem_ros_node::pub_full_state_()
{
    dv_interfaces::full_state msg;

    // To jest dokładnie wejście, które wchodzi do fizyki (czyli "applied to actuators")
    Input in(
        torque_command_to_invert_rr_,
        torque_command_to_invert_rl_,
        torque_command_to_invert_fr_,
        torque_command_to_invert_fl_,
        steer_command_request
    );

    Log_Info_full info = log_info_full(state_, in, P_, step_number_);

    // --- meta ---
    msg.time        = static_cast<float>(info.time);
    msg.step_number = static_cast<int32_t>(step_number_);

    // --- pose ---
    msg.x        = static_cast<float>(info.x);
    msg.y        = static_cast<float>(info.y);
    msg.yaw      = static_cast<float>(info.yaw);
    msg.yaw_rate = static_cast<float>(info.yaw_rate);

    // --- vel/acc ---
    msg.vx = static_cast<float>(info.vx);
    msg.vy = static_cast<float>(info.vy);
    msg.ax = static_cast<float>(info.ax);
    msg.ay = static_cast<float>(info.ay);

    // --- torques actually realized in state / model (z info) ---
    msg.torque_fl = static_cast<float>(info.torque_fl);
    msg.torque_fr = static_cast<float>(info.torque_fr);
    msg.torque_rl = static_cast<float>(info.torque_rl);
    msg.torque_rr = static_cast<float>(info.torque_rr);

    // --- torque requests sent to inverters (z pipeline SEND) ---
    msg.torque_request_fl = static_cast<float>(torque_command_to_invert_fl_);
    msg.torque_request_fr = static_cast<float>(torque_command_to_invert_fr_);
    msg.torque_request_rl = static_cast<float>(torque_command_to_invert_rl_);
    msg.torque_request_rr = static_cast<float>(torque_command_to_invert_rr_);

    // --- wheel speeds ---
    msg.omega_fl = static_cast<float>(info.omega_fl);
    msg.omega_fr = static_cast<float>(info.omega_fr);
    msg.omega_rl = static_cast<float>(info.omega_rl);
    msg.omega_rr = static_cast<float>(info.omega_rr);

    // --- steering ---
    msg.rack_angle         = static_cast<float>(info.rack_angle);
    msg.delta_left         = static_cast<float>(info.delta_left);
    msg.delta_rigth        = static_cast<float>(info.delta_rigth); // literówka wg msg
    msg.rack_angle_request = static_cast<float>(info.rack_angle_request);

    // --- forces ---
    msg.fx_fl = static_cast<float>(info.fx_fl);
    msg.fx_fr = static_cast<float>(info.fx_fr);
    msg.fx_rl = static_cast<float>(info.fx_rl);
    msg.fx_rr = static_cast<float>(info.fx_rr);

    msg.fy_fl = static_cast<float>(info.fy_fl);
    msg.fy_fr = static_cast<float>(info.fy_fr);
    msg.fy_rl = static_cast<float>(info.fy_rl);
    msg.fy_rr = static_cast<float>(info.fy_rr);

    msg.fz_fl = static_cast<float>(info.fz_fl);
    msg.fz_fr = static_cast<float>(info.fz_fr);
    msg.fz_rl = static_cast<float>(info.fz_rl);
    msg.fz_rr = static_cast<float>(info.fz_rr);

    // --- slips ---
    msg.slip_angle_fl   = static_cast<float>(info.slip_angle_fl);
    msg.slip_angle_fr   = static_cast<float>(info.slip_angle_fr);
    msg.slip_angle_rl   = static_cast<float>(info.slip_angle_rl);
    msg.slip_angle_rr   = static_cast<float>(info.slip_angle_rr);
    msg.slip_angle_body = static_cast<float>(info.slip_angle_body);

    msg.kappa_fl = static_cast<float>(info.kappa_fl);
    msg.kappa_fr = static_cast<float>(info.kappa_fr);
    msg.kappa_rl = static_cast<float>(info.kappa_rl);
    msg.kappa_rr = static_cast<float>(info.kappa_rr);

    // --- aero/energy ---
    msg.total_drag      = static_cast<float>(info.total_drag);
    msg.total_downforce = static_cast<float>(info.total_downforce);
    msg.Power_total     = static_cast<float>(info.Power_total);

    // --- sim param ---
    msg.step_dt = static_cast<float>(P_.get("simulation_time_step"));

    pub_log_full_.publish(msg);
}

// ============================================================================
//  Car marker
// ============================================================================
void Simulation_lem_ros_node::publish_bolid_marker_()
{
    visualization_msgs::Marker car;
    car.header.frame_id = "bolide_true";
    car.header.stamp    = ros::Time::now();
    car.ns   = "bolide";
    car.id   = 0;
    car.type = visualization_msgs::Marker::CUBE;
    car.action = visualization_msgs::Marker::ADD;

    car.scale.x = 2.8;
    car.scale.y = 1.5;
    car.scale.z = 1.5;

    car.pose.position.x = 0.0;
    car.pose.position.y = 0.0;
    car.pose.position.z = 0.75;
    car.pose.orientation.w = 1.0;

    car.color.r = 0.0f;
    car.color.g = 0.9f;
    car.color.b = 0.2f;
    car.color.a = 1.0f;

    car.lifetime = ros::Duration(0.1);

    pub_marker_bolid_.publish(car);
}

} // namespace lem_dynamics_sim_


#include "sim_loop.hpp"
#include <iostream>   // <<< diagnostyka
#include <exception>  // <<< diagnostyka
#include <algorithm>  // std::clamp
#include <sstream>
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

    static inline void update_top_abs(std::vector<double>& v, double x, std::size_t N = 10)
    {
        const double ax = std::abs(x);

        // jeśli jeszcze nie ma N elementów -> wrzuć i posortuj
        if (v.size() < N) {
            v.push_back(x);
            std::sort(v.begin(), v.end(),
                    [](double a, double b){ return std::abs(a) > std::abs(b); });
            return;
        }

        // jeśli x nie jest większe niż najmniejsze z TOP -> ignore
        double smallest_abs = std::abs(v.back());
        if (ax <= smallest_abs) return;

        // podmień najmniejsze i ponownie posortuj
        v.back() = x;
        std::sort(v.begin(), v.end(),
                [](double a, double b){ return std::abs(a) > std::abs(b); });
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
    std::cout << "[INIT] Resetting control/PIDs state..." << std::endl;

    ///


    // 3) Interwały krokowe (po wczytaniu P_)
    std::cout << "[INIT] Computing step intervals from parameters..." << std::endl;
    try {
        compute_step_intervals_from_params_();
        std::cout << "[INIT][OK] Step intervals computed." << std::endl;

        // 6) Fazy (rozstrzelenie w obrębie okresu)
        phase_camera_shoot_          = pick_phase_(step_of_camera_shoot_);
        phase_lidar_shoot_           = pick_phase_(step_of_lidar_shoot_);
        phase_wheel_encoder_reading_ = pick_phase_(step_of_wheel_encoder_reading_);
        phase_ins_reading_           = pick_phase_(step_of_ins_reading_);

        // READ / SEND
        phase_torque_apply_ = pick_phase_(step_of_control_input_read_);
        phase_steer_apply_  = pick_phase_(step_of_steer_input_sending_);
        phase_gps_speed_reading_    = pick_phase_(step_gps_speed_reading_);
        phase_control_input_read_    = pick_phase_(step_of_control_input_read_);
        phase_dv_board_data_publishing_ = pick_phase_(step_of_dv_board_data_publishing_);
        phase_imu_publishing_ = pick_phase_(step_of_imu_publishing_);
       // phase_wheel_encoder_publishing_ = pick_phase_(step_of_wheel_encoder_publishing_); not used currently
        phase_steer_publishing_ = pick_phase_(step_of_steer_publishing_);


        std::cout << "[PHASES] cam="    << phase_camera_shoot_
                << " lidar="          << phase_lidar_shoot_
                << " enc="            << phase_wheel_encoder_reading_
                << " ins="            << phase_ins_reading_
                << " torque_apply="   << phase_torque_apply_
                << " steer_apply="    << phase_steer_apply_
                << " gps_speed="      << phase_gps_speed_reading_
                << " control_read="   << phase_control_input_read_
                << " dv_board_pub="   << phase_dv_board_data_publishing_
                << " imu_pub="        << phase_imu_publishing_
               // << " wheel_pub="      << phase_wheel_encoder_publishing_
                << " steer_pub="      << phase_steer_publishing_
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
        sub_mpc_debug_ = nh.subscribe<dv_interfaces::MPCDebug>(
            "/control/mpc_debug", 1, &Simulation_lem_ros_node::mpc_debug_callback_, this);
        pub_ins_   = nh.advertise<nav_msgs::Odometry>("/ins/pose", 1);
        pub_imu_   = nh.advertise<dv_interfaces::Imu>("/dv_board/imu", 1);
        pub_dv_board_data_ = nh.advertise<dv_interfaces::DV_board>("/dv_board/data", 1);
        pub_steer_ = nh.advertise<dv_interfaces::Steer>("/steer", 1);
        pub_cones_ = nh.advertise<dv_interfaces::Cones>("/dv_cone_detector/cones", 1);
        pub_lidar_cones_ = nh.advertise<dv_interfaces::Cones>("/dv_cone_detector/cones", 1);
        pub_markers_cones_gt_  = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_gt", 1,true);
        pub_markers_cones_vis_ = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_vis", 1);
        pub_markers_cones_lidar_ = nh.advertise<visualization_msgs::MarkerArray>("/viz/cones_lidar", 1);
        pub_log_full_ = nh.advertise<dv_interfaces::full_state>("/debug/full_log_info", 1);
        pub_marker_bolid_ = nh.advertise<visualization_msgs::Marker>("/viz/bolide_model", 1);
        pub_gg_sphere_marker_ = nh.advertise<visualization_msgs::Marker>("/simulation/gg_sphere", 1);
        std::cout << "[INIT][OK] ROS I/O ready." << std::endl;
        
        auto check_pub = [&](const char* name, const ros::Publisher& p){
            if (!p) {
                std::cerr << "[INIT][FAIL] Publisher invalid right after advertise(): " << name << std::endl;
                ROS_ERROR("Publisher invalid right after advertise(): %s", name);
            }
        };
        check_pub("pub_ins_",               pub_ins_);
        check_pub("pub_imu_",               pub_imu_);
        check_pub("pub_dv_board_data_",     pub_dv_board_data_);
        check_pub("pub_steer_",             pub_steer_);
        check_pub("pub_cones_",             pub_cones_);
        check_pub("pub_lidar_cones_",       pub_lidar_cones_);
        check_pub("pub_markers_cones_gt_",  pub_markers_cones_gt_);
        check_pub("pub_markers_cones_vis_", pub_markers_cones_vis_);
        check_pub("pub_markers_cones_lidar_", pub_markers_cones_lidar_);

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

    // 6) Inicjalizacja logowania metryk jazdy
    if (!log_file.empty()) {
        metrics_log_file_path_ = log_file;
        const auto pos = metrics_log_file_path_.rfind(".csv");
        if (pos != std::string::npos) {
            metrics_log_file_path_.replace(pos, 4, "_metrics.csv");
        } else {
            metrics_log_file_path_ += "_metrics.csv";
        }
    } else {
        metrics_log_file_path_.clear(); // metryki wyłączone jeśli log_file pusty
    }

   

    // 7) Kolejki puste na start
    camera_queue_.clear();
    timestamp_queue_.clear();
    lidar_queue_.clear();
    lidar_timestamp_queue_.clear();

    std::cout << "[INIT][DONE] Node constructed successfully." << std::endl;

    ROS_WARN_STREAM("Init yaw=" << state_.yaw << " vy=" << state_.vy);

    // --- Traction control PID init (drive/brake) ---
    {
        PIDParams drive{};
        drive.Kp = P_.get("pid_traction_p");
        drive.Ki = P_.get("pid_traction_i");
        drive.Kd = P_.get("pid_traction_d");

        // Limity TC wg configu
        drive.saturation_upper = P_.get("pid_traction_max_drive");
        drive.saturation_lower = P_.get("pid_traction_min_drive");

        drive.anti_windup_gain = P_.get("pid_traction_anti_windup_gain_drive");
        drive.leak_time_scale  = P_.get("pid_traction_leak_time_scale_drive");
        traction_control_pid_drive_.set_params(drive);
        traction_control_pid_drive_.reset();

        PIDParams brake{};
        brake.Kp = P_.get("pid_traction_p");
        brake.Ki = P_.get("pid_traction_i");
        brake.Kd = P_.get("pid_traction_d");

        brake.saturation_upper = P_.get("pid_traction_max_brake");
        brake.saturation_lower = P_.get("pid_traction_min_brake");

        brake.anti_windup_gain = P_.get("pid_traction_anti_windup_gain_brake");
        brake.leak_time_scale  = P_.get("pid_traction_leak_time_scale_brake");
        traction_control_pid_brake_.set_params(brake);
        traction_control_pid_brake_.reset();
    }

    // --- Kalman init (jeśli używany) ---
    {
        // KalmanFilter ma konstruktor z ParamBank; tworzymy i resetujemy go po wczytaniu P_
        kalman_filter_ = KalmanFilter(P_);

        // opcjonalnie: ins_mode z ROS param (gauss|kalman)
        // (jeśli param nie istnieje, zostaje domyślne z .hpp)
        std::string mode;
        if (nh.getParam("ins_mode", mode)) {
            ins_mode_ = mode;
        }
        bool use_llc;
        if (nh.getParam("low_level_controlers", use_llc)) {
            lov_level_control_on = false; // for now no tc as it needs some changes

        }
    }

    {
        bool use_lidar_;
        if (nh.getParam("use_lidar", use_lidar_)) {
            use_lidar = use_lidar_;
        }
    }
    int sim_time_ ;
    if(nh.getParam("sim_time", sim_time_)){
        sim_time = sim_time_;
    }

    std::cout << "[INIT] sim_time = " << sim_time_
            << "  ( <0 => infinite , >=0 => stop after sim_time seconds )"
            << std::endl;

    ROS_WARN_STREAM("[INIT] sim_time=" << sim_time);
}

// ====== Destruktor ======
Simulation_lem_ros_node::~Simulation_lem_ros_node() {
    std::cout << "[DTOR] Saving ride metrics..." << std::endl;
    log_metric_of_ride_data_();

}

// ====== Interfejs publiczny ======
void Simulation_lem_ros_node::step() {
    try {
        read_control_by_dv_board_if_due();
        read_wheel_encoder_if_due_();
        read_ins_if_due_();
        shoot_camera_or_enqueue_if_due_();
        shoot_lidar_or_enqueue_if_due_();

        send_to_ts_if_due();
        send_steer_to_maxon_if_due_();
        publish_ready_camera_frames_from_queue_();
        publish_ready_lidar_frames_from_queue_();

        euler_sim_timestep(state_, Input(torque_command_to_invert_, steer_command_to_maxon_), P_);


        ++step_number_;

        pub_full_state_();
        publish_bolid_marker_();
        publish_bolid_tf_true();

        publish_dv_board_data_if_due_();
        publish_imu_if_due_();
       // publish_wheel_encoder_if_due_();
        publish_steer_if_due_();

        // ======================================================
        // STOP AFTER sim_time_ seconds (if enabled)
        // ======================================================
        if (sim_time >= 0)
        {
            const double dt = P_.get("simulation_time_step");
            const double t_sim = step_number_ * dt;

            if (t_sim >= static_cast<double>(sim_time))
            {
                ROS_WARN_STREAM("[SIM_LOOP] sim_time reached: " << t_sim
                                << " / " << sim_time << " s -> shutting down ROS node.");
                ros::shutdown();
                return;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "[STEP][FAIL] Exception in simulation step. what(): " << e.what() << std::endl;
        throw;
    }
}



// dv board reads control topic at fixed cadence
void Simulation_lem_ros_node::read_control_by_dv_board_if_due()
{
    if( is_due(step_number_, step_of_control_input_read_, phase_control_input_read_) )
    last_input_read_by_dv_board = last_input_cached;
}

void Simulation_lem_ros_node::read_wheel_encoder_if_due_()
{
    if (step_of_wheel_encoder_reading_ <= 0) return;
    if (!is_due(step_number_, step_of_wheel_encoder_reading_, phase_wheel_encoder_reading_)) return;

    const double R = P_.get("R");

    wheel_speed_read_right = state_.omega_rr * R;
    wheel_speed_read_left  = state_.omega_rl * R;

}

void Simulation_lem_ros_node::read_steer_by_orin_if_due_()
{
    if( is_due(step_number_, step_of_steer_input_sending_, phase_steer_apply_) )
    {
        steer_command_to_maxon_ = last_input_cached.steeringAngle_rad;
    }


}


// ====== ROS callback ======
// caching last requested input from control
void Simulation_lem_ros_node::dv_control_callback(const dv_interfaces::Control::ConstPtr& msg)
{
    last_input_cached = *msg;
}

// ====== Pomocnicze ======
void Simulation_lem_ros_node::compute_step_intervals_from_params_() {
    const double dt = P_.get("simulation_time_step");
    std::cout << "[INTERVALS] dt=" << dt << std::endl;

    // sensor cadences
    step_of_camera_shoot_          = std::max(1, (int)std::round(1.0 / P_.get("frames_per_second") / dt));
    step_of_lidar_shoot_           = std::max(1, (int)std::round(1.0 / P_.get("lidar_frames_per_second") / dt));
    step_of_wheel_encoder_reading_ = std::max(1, (int)std::round(P_.get("wheel_encoder_reading_time_step") / dt));
    step_of_ins_reading_           = std::max(1, (int)std::round(1.0 / P_.get("ins_frequancy") / dt));
    step_gps_speed_reading_ = std::max(1, (int)std::round(1.0 / P_.get("gps_speed_frequancy") / dt));
    step_imu_reading_            = std::max(1, (int)std::round(1.0 / P_.get("acc_frequancy") / dt));
   

    // READ/SEND pipeline cadences (must exist in .hpp)
    step_of_control_input_read_       = std::max(1, (int)std::round(P_.get("control_to_dv_boad_read_time_step") / dt));
    step_of_steer_input_sending_      = std::max(1, (int)std::round(P_.get("dv_board_to_maxon_time_step") / dt));
    step_number_torque_input_sending_ = std::max(1, (int)std::round(P_.get("dv_board_tractive_system_time_step") / dt));

    // odometry related sensor publishing cadence
    step_of_dv_board_data_publishing_ = std::max(1, (int)std::round(P_.get("dv_board_data_publishing_time_step") / dt));
    step_of_imu_publishing_ = std::max(1, (int)std::round(P_.get("imu_publishing_time_step") / dt));
   // step_of_wheel_encoder_publishing_ = std::max(1, (int)std::round(P_.get("wheel_encoder_publishing_time_step") / dt));
    step_of_steer_publishing_ = std::max(1, (int)std::round(P_.get("steer_publishing_time_step") / dt));

    std::cout << "[INTERVALS] cam="      << step_of_camera_shoot_
              << " lidar="              << step_of_lidar_shoot_
              << " enc="                << step_of_wheel_encoder_reading_
              << " ins="                << step_of_ins_reading_
              << " gps="                << step_gps_speed_reading_
              << " ctrl_read="          << step_of_control_input_read_
              << " steer_send="         << step_of_steer_input_sending_
              << " torque_send="        << step_number_torque_input_sending_
              << " dv_board_pub="       << step_of_dv_board_data_publishing_
              << " imu_pub="            << step_of_imu_publishing_
             // << " wheel_enc_pub="      << step_of_wheel_encoder_publishing_
              << " steer_pub="          << step_of_steer_publishing_

              << std::endl;


}

// dv board reads control topic at fixed cadence
void Simulation_lem_ros_node::read_ins_if_due_()
{
    const bool due_ins = (step_of_ins_reading_ > 0) &&
                         is_due(step_number_, step_of_ins_reading_, phase_ins_reading_);

    const bool due_gps = (step_gps_speed_reading_ > 0) &&
                         is_due(step_number_, step_gps_speed_reading_, phase_gps_speed_reading_);

    const bool due_imu = (step_imu_reading_ > 0) &&
                         is_due(step_number_, step_imu_reading_, 0);

    const int calib_steps = std::max(
        1,
        static_cast<int>(P_.get("calibration_time") / P_.get("simulation_time_step"))
    );
    const bool calibrated = (step_number_ > calib_steps);

    // Jeśli nic nie robimy w tej iteracji – wyjdź
    if (!due_ins && !due_gps && !due_imu) return;


    // ----------------------------
    // IMU (cache)
    // ----------------------------
    
    if (due_imu) {
        ImuMeasurement imu_meas{};

        const double dt = P_.get("simulation_time_step")*step_imu_reading_;

        // random walk bias update
        sim_b_g  += P_.get("gyro_bias_rw") * std::sqrt(dt) * random_noise_generator_();
        sim_b_ax += P_.get("acc_bias_rw")  * std::sqrt(dt) * random_noise_generator_();
        sim_b_ay += P_.get("acc_bias_rw")  * std::sqrt(dt) * random_noise_generator_();

        // measured IMU
        imu_meas.yaw_rate = state_.yaw_rate + sim_b_g
                            + P_.get("gyro_noise_std") * random_noise_generator_();
        imu_meas.ax = state_.prev_ax + sim_b_ax
                    + P_.get("acc_noise_std") * random_noise_generator_();
        imu_meas.ay = state_.prev_ay + sim_b_ay
                    + P_.get("acc_noise_std") * random_noise_generator_();


        double imu_smoter_yaw_rate = state_.yaw_rate + P_.get("gyro_noise_smoter_std") * random_noise_generator_();
        double imu_smoter_ax = state_.prev_ax + P_.get("acc_noise_smoter_std") * random_noise_generator_();
        double imu_smoter_ay = state_.prev_ay + P_.get("acc_noise_smoter_std") * random_noise_generator_();
       

        last_imu_ = imu_meas;
        has_last_imu_ = true;


        if (ins_mode_ == "kalman") {
            kalman_filter_.predict(imu_meas, dt);
        }
    }
    

    // ----------------------------
    // GPS (cache) + INS velocity from GPS ALWAYS
    // ----------------------------
    if (due_gps) {
        GpsMeasurement gps_meas{};

        const double vx_global = state_.vx * std::cos(state_.yaw) - state_.vy * std::sin(state_.yaw);
        const double vy_global = state_.vx * std::sin(state_.yaw) + state_.vy * std::cos(state_.yaw);

        gps_meas.x   = state_.x + P_.get("gps_position_noise") * random_noise_generator_();
        gps_meas.y   = state_.y + P_.get("gps_position_noise") * random_noise_generator_();
        gps_meas.vx  = vx_global + P_.get("gps_speed_noise")   * random_noise_generator_();
        gps_meas.vy  = vy_global + P_.get("gps_speed_noise")   * random_noise_generator_();
        double yaw = state_.yaw + P_.get("gps_yaw_noise") * random_noise_generator_();
        unwrap_angle(yaw);
        gps_meas.yaw = yaw;
        
        

        last_gps_ = gps_meas;
        has_last_gps_ = true;

        ins_data_to_be_published_.vx = gps_meas.vx;
        ins_data_to_be_published_.vy = gps_meas.vy;


	
        if (ins_mode_ == "kalman") {
            
            kalman_filter_.update_gps(gps_meas);

        }
        
    }

    // ----------------------------
    // INS publish tick
    // ----------------------------
    if (!due_ins) return;

    
    	if (!has_last_gps_) {
           
          	// jeszcze nie przyszedł żaden GPS
            ins_data_to_be_published_.vx = 0.0;
            ins_data_to_be_published_.vy = 0.0;
        }


    if (ins_mode_ == "gauss") {
        ins_data_to_be_published_.x   = state_.x + P_.get("pose_noise") * random_noise_generator_();
        ins_data_to_be_published_.y   = state_.y + P_.get("pose_noise") * random_noise_generator_();
        double yaw = state_.yaw + P_.get("orientation_noise") * random_noise_generator_();
        unwrap_angle(yaw);
        ins_data_to_be_published_.yaw = yaw;
        ins_data_to_be_published_.yaw_rate = state_.yaw_rate +P_.get("rotation_noise") * random_noise_generator_();

    }  else if (ins_mode_ == "kalman") {
 
        ins_data_to_be_published_.x = kalman_filter_.get_state().x;
        ins_data_to_be_published_.y = kalman_filter_.get_state().y;
        ins_data_to_be_published_.yaw = kalman_filter_.get_state().yaw;
        ins_data_to_be_published_.yaw_rate = kalman_filter_.get_state().yaw_rate;

    } 

    if (calibrated) publish_ins_(ins_data_to_be_published_);
    last_ins_data_already_published_ = ins_data_to_be_published_;
    if (calibrated) publish_bolid_tf_ins(ins_data_to_be_published_);
}



void Simulation_lem_ros_node::shoot_camera_or_enqueue_if_due_()
{
    if (step_of_camera_shoot_ <= 0) return;
    if (!is_due(step_number_, step_of_camera_shoot_, phase_camera_shoot_)) return;

    if(use_lidar) return; // if lidar enabled, camera is disabled

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

void Simulation_lem_ros_node::shoot_lidar_or_enqueue_if_due_()
{
    if (step_of_lidar_shoot_ <= 0) return;
    if (!is_due(step_number_, step_of_lidar_shoot_, phase_lidar_shoot_)) return;

    if(!use_lidar) return; // if lidar disabled, do not shoot

    Track detection = shoot_a_frame_lidar(track_global_, P_, state_);
    const double dt = P_.get("simulation_time_step");
    const double lidar_exec_time = sample_lidar_exec_time_();
    const int processing_steps = std::max(0, (int)std::round(lidar_exec_time / dt));

    LidarTask task;
    task.ready_step = step_number_ + processing_steps;
    task.frame      = std::move(detection);

    if ((int)lidar_queue_.size() >= 3) {
        lidar_queue_.pop_front();
        lidar_timestamp_queue_.pop_front();
    }

    lidar_queue_.push_back(std::move(task));
    lidar_timestamp_queue_.push_back(ros::Time::now());
}


void Simulation_lem_ros_node::publish_ready_camera_frames_from_queue_() {

    if(use_lidar) return; // if lidar enabled, camera is disabled
    while (!camera_queue_.empty() && camera_queue_.front().ready_step <= step_number_) {
        const auto& task = camera_queue_.front();
        const auto& timestamp = timestamp_queue_.front();
        publish_cones_(task.frame, timestamp);
        publish_cones_vision_markers_(task.frame, timestamp);
        camera_queue_.pop_front();
        timestamp_queue_.pop_front();
    }
}

void Simulation_lem_ros_node::publish_ready_lidar_frames_from_queue_() {
    if(!use_lidar) return; // if lidar disabled, do not publish

    while (!lidar_queue_.empty() && lidar_queue_.front().ready_step <= step_number_) {
        const auto& task = lidar_queue_.front();
        const auto& timestamp = lidar_timestamp_queue_.front();
        publish_lidar_cones_(task.frame, timestamp);
        publish_cones_lidar_markers_(task.frame, timestamp);
        lidar_queue_.pop_front();
        lidar_timestamp_queue_.pop_front();
    }
}

// sending torque to tractive system if due - dv_board communicates with tractive system at fixed cadence
void Simulation_lem_ros_node::send_to_ts_if_due()
{


    // 2) TS update only at its cadence
    if (!is_due(step_number_, step_number_torque_input_sending_, phase_torque_apply_)) {
        return;
    }

    // msg: move_type: 0 = TORQUE_PERCENTAGE, 1 = SPEED_KMH
    const bool speed_mode = last_input_read_by_dv_board.move_type;
    const double Ts = P_.get("simulation_time_step") * step_number_torque_input_sending_;


    if (!speed_mode) {
        // TORQUE_PERCENTAGE: movement is [%] (expected -100..100)
        const double torque_percentage = static_cast<double>(last_input_read_by_dv_board.movement);
        const double speed_current = static_cast<double>(last_input_read_by_dv_board.current_speed);
        double torque_temp = std::clamp(torque_percentage, -100.0, 100.0);
        torque_temp = (torque_temp * P_.get("max_torque")) / 100.0;

        const double wheel_speed_drive_on = speed_current*(1 + P_.get("slip_drive_on"));
        const double wheel_speed_drive_off = speed_current*(1 + P_.get("slip_drive_off"));
        const double wheel_speed_brake_on =  speed_current*(1 + P_.get("slip_brake_on"));
        const double wheel_speed_brake_off =  speed_current*(1 + P_.get("slip_brake_off"));

        const double speed_target_drive = speed_current * (1 + P_.get("target_slip_drive"));
        const double speed_target_brake = speed_current * (1 + P_.get("target_slip_brake"));


        //  Sprawdzenie wyłacznie TC
        // warunek wyłączenia TC dla napędu
        if( 
            wheel_speed_read_left < wheel_speed_drive_off &&
            wheel_speed_read_right < wheel_speed_drive_off )
        {
            traction_control_pid_drive_.update(0.0, Ts,false); // teraz wyłączamy TC -> integrator robi leak
        }

        if( 
            wheel_speed_read_left  > wheel_speed_brake_off &&
            wheel_speed_read_right > wheel_speed_brake_off )
        {
            traction_control_pid_brake_.update(0.0, Ts,false); // teraz wyłączamy TC -> integrator robi leak
        }



        // TRACTION CONTROL - DRIVE

        if( torque_temp > 0 ) // tylko przy dodatnim momencie
        {

            const double error =  (std::max(wheel_speed_read_left, wheel_speed_read_right) - speed_target_drive) / std::max( speed_current , 2.0) ;
            // warunek włączenia TC dla napędu

            if(traction_control_pid_drive_.is_active())
            {
                traction_control_pid_drive_.update(error, Ts,true);
                const double tc_output = traction_control_pid_drive_.get_output();
                if(lov_level_control_on) torque_temp = std::max(0.0,std::min(torque_temp, 2*tc_output));
            }

            else if( wheel_speed_read_left  > wheel_speed_drive_on || wheel_speed_read_right > wheel_speed_drive_on )
            {
                
                traction_control_pid_drive_.update(error, Ts,true);
                const double tc_output = traction_control_pid_drive_.get_output();
                if(lov_level_control_on) torque_temp = std::max(0.0,std::min(torque_temp, 2*tc_output));
            }
            // jeśli włączony to aktualizuj PID 
            
        }
        
        // TRACTION CONTROL - BRAKE 
        if( torque_temp < 0 ) // tylko przy ujemnym momencie
        {
            const double error =  (std::min(wheel_speed_read_left, wheel_speed_read_right) - speed_target_brake)/std::max( speed_current , 2.0) ;
            // warunek włączenia TC dla hamowania
             // jeśli włączony to aktualizuj PID 
             if(traction_control_pid_brake_.is_active())
             {
                 traction_control_pid_brake_.update(error, Ts,true);
                 const double tc_output = traction_control_pid_brake_.get_output();
                if(lov_level_control_on) torque_temp = std::min(0.0,std::max(torque_temp, 2*tc_output));
             }

            
            else if( wheel_speed_read_left  < wheel_speed_brake_on || wheel_speed_read_right < wheel_speed_brake_on )
            {
                
                traction_control_pid_brake_.update(error, Ts,true);
                const double tc_output = traction_control_pid_brake_.get_output();
                if(lov_level_control_on) torque_temp = std::min(0.0,std::max(torque_temp, 2*tc_output));
            }
           
        }

         // =========================================================
        // TC active time accounting (NEW)
        // =========================================================
        const bool tc_active_now =
            traction_control_pid_drive_.is_active() ||
            traction_control_pid_brake_.is_active();

        if (tc_active_now) {
            time_tc_active_ += Ts;  // [s] czas aktywnego TC
        }

        torque_command_to_invert_ = torque_temp;
        return;
     
    }
    // SPEED_KMH: movement in km/h
    // trapezoidal I
    double error = static_cast<double>(last_input_read_by_dv_board.movement) - (wheel_speed_read_right + wheel_speed_read_left)/2 ;
    prev_I_speed_pid += (error + prev_error_speed_pid) * 0.5 * Ts;

    double u_pid =
        P_.get("pid_speed_p") * error +
        P_.get("pid_speed_i") * prev_I_speed_pid +
        P_.get("pid_speed_d") * (error - prev_error_speed_pid) / Ts;

    prev_error_speed_pid = error;

    u_pid = std::clamp(u_pid, P_.get("pid_speed_min"), P_.get("pid_speed_max"));

    // PID output -> torque
    torque_command_to_invert_ = (u_pid * P_.get("max_torque")) / P_.get("pid_speed_scale");

    return;
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


    // odom_msg.pose.pose.position.x = state_.x;
    // odom_msg.pose.pose.position.y = state_.y;
    // odom_msg.pose.pose.position.z = 0.0;

    // const double vx_global = state_.vx * std::cos(state_.yaw) - state_.vy * std::sin(state_.yaw);
    // const double vy_global = state_.vx * std::sin(state_.yaw) + state_.vy * std::cos(state_.yaw);

    // tf2::Quaternion q;
    // q.setRPY(0.0, 0.0, ins.yaw);
    // odom_msg.pose.pose.orientation.x = q.x();
    // odom_msg.pose.pose.orientation.y = q.y();
    // odom_msg.pose.pose.orientation.z = q.z();
    // odom_msg.pose.pose.orientation.w = q.w();

    // odom_msg.twist.twist.linear.x  = vx_global;
    // odom_msg.twist.twist.linear.y  = vy_global;
    // odom_msg.twist.twist.angular.z = state_.yaw_rate;

   

    pub_ins_.publish(odom_msg);
}

void Simulation_lem_ros_node::publish_cones_(const Track& cones, ros::Time timestamp){
    if(use_lidar) return; // if lidar enabled, camera is disabled, so do not publish camera cones
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

void Simulation_lem_ros_node::publish_lidar_cones_(const Track& cones, ros::Time timestamp){

    if(!use_lidar) return; // if lidar disabled, do not publish
    if (!pub_lidar_cones_) {
        std::cerr << "[PUB][ERROR] pub_lidar_cones_ invalid; NOT publishing lidar cones." << std::endl;
        ROS_ERROR("pub_lidar_cones_ invalid; not publishing lidar cones");
        return;
    }

    dv_interfaces::Cones cones_msg;
    cones_msg.header.stamp = timestamp;
    cones_msg.header.frame_id = "os_sensor";
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
    pub_lidar_cones_.publish(cones_msg);
}

double Simulation_lem_ros_node::sample_vision_exec_time_() const {
    const double mu  = P_.get("mean_time_of_vision_execuction");
    const double var = P_.get("var_of_vision_time_execution");

    static thread_local std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<double> normal(mu, std::sqrt(std::max(0.0, var)));

    const double exec_time = normal(rng);
    return std::abs(exec_time); // czas wykonania nie może być ujemny
}

double Simulation_lem_ros_node::sample_lidar_exec_time_() const {
    const double mu  = P_.get("lidar_execution_time_mean");
    const double var = P_.get("lidar_execution_time_var");

    static thread_local std::mt19937 rng{std::random_device{}()};
    std::normal_distribution<double> normal(mu, std::sqrt(std::max(0.0, var)));

    const double exec_time = normal(rng);
    return std::abs(exec_time); // czas wykonania nie może być ujemny
}



void Simulation_lem_ros_node::publish_cones_vision_markers_(
    const Track& det, const ros::Time& acquisition_stamp)
{
    if(use_lidar) return; // if lidar enabled, camera is disabled, so do not publish vision markers
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

void Simulation_lem_ros_node::publish_cones_lidar_markers_(
    const Track& det, const ros::Time& acquisition_stamp)
{
    if(!use_lidar) return; // if lidar disabled, do not publish lidar markers
    if (!pub_markers_cones_lidar_) {
        ROS_ERROR("pub_markers_cones_lidar_ invalid; not publishing lidar markers");
        return;
    }

    visualization_msgs::MarkerArray arr;

    // ======================================================
    // 1) Usuń poprzednie markery lidaru (TYLKO SWOJE)
    // ======================================================
    for (int id = 400; id < 400 + last_lidar_frame_size_; id++)
    {
        visualization_msgs::Marker del;
        del.header.frame_id = "os_sensor";
        del.header.stamp    = acquisition_stamp;
        del.ns              = "cones_lidar";
        del.id              = id;
        del.action          = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(del);
    }

    // ======================================================
    // 2) Dodaj nowe markery
    // ======================================================
    const double fps = std::max(1e-3, P_.get("lidar_frames_per_second"));
    const ros::Duration lifetime(5.0 / fps);

    int id = 400;
    for (const auto& c : det.cones)
    {
        std_msgs::ColorRGBA col = color_from_class_lidar(c.color, 0.75f);

        visualization_msgs::Marker m;
        m.header.frame_id = "os_sensor";
        m.header.stamp    = ros::Time::now();
        m.ns              = "cones_lidar";
        m.id              = id++;
        m.type            = visualization_msgs::Marker::SPHERE;
        m.action          = visualization_msgs::Marker::ADD;

        m.pose.position.x = c.x;
        m.pose.position.y = c.y;
        m.pose.position.z = c.z ;
        m.pose.orientation.w = 1.0;

        m.scale.x = 0.22;
        m.scale.y = 0.22;
        m.scale.z = 0.22;

        m.color    = col;
        m.lifetime = lifetime;
        m.frame_locked = false;

        arr.markers.push_back(std::move(m));
    }

    // ======================================================
    // 3) Zapisz ile markerów było w aktualnej ramce
    // ======================================================
    last_lidar_frame_size_ = det.cones.size();

    pub_markers_cones_lidar_.publish(arr);
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
    Log_Info_full info = log_info_full(state_, Input(torque_command_to_invert_, steer_command_to_maxon_), P_, step_number_);

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

    msg.torque = info.torque;
    msg.torque_left = info.torque_left;
    msg.torque_right = info.torque_right;
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
    msg.torque_request = info.torque_request;

    msg.step_dt = P_.get("simulation_time_step");

    const double kappa_max_abs_signed =
    (std::abs(msg.kappa_rr) > std::abs(msg.kappa_rl)) ? msg.kappa_rr : msg.kappa_rl;

    update_top_abs(ten_biggest_slip_ratio_, kappa_max_abs_signed, 10);
    update_top_abs(ten_biggest_beta_angle_, msg.slip_angle_body, 10);

    const double beta_thresh = 9.0 ;
    if (std::abs(msg.slip_angle_body) > beta_thresh) {
        time_beta_over_9_ += msg.step_dt;
    }

    pub_log_full_.publish(msg);

     // ==========================================================
    // [NOWE] RYSOWANIE KROPLI G-G (MARKER)
    // ==========================================================
    visualization_msgs::Marker gg_sphere;
    gg_sphere.header.frame_id = "gg_dashboard"; // 
    gg_sphere.header.stamp = ros::Time::now();
    gg_sphere.ns = "gg_current_accel";
    gg_sphere.id = 1; // Musi być inny niż ID obwiedni!
    gg_sphere.type = visualization_msgs::Marker::SPHERE;
    gg_sphere.action = visualization_msgs::Marker::ADD;

    // Współrzędne na wykresie:
    // Oś X wykresu = Przyspieszenie boczne (ay)
    // Oś Y wykresu = Przyspieszenie wzdłużne (ax)
    gg_sphere.pose.position.x = info.ay; 
    gg_sphere.pose.position.y = info.ax;
    gg_sphere.pose.position.z = 0.0;
    
    // Brak rotacji
    gg_sphere.pose.orientation.w = 1.0; 

    // Rozmiar "kropli" (np. 15 cm średnicy na wykresie)
    gg_sphere.scale.x = 1.15;
    gg_sphere.scale.y = 1.15;
    gg_sphere.scale.z = 1.15; // Byłaby to kula, ale patrzymy z góry

    // Kolor - np. jaskrawy czerwony, żeby odcinał się od szarej obwiedni
    gg_sphere.color.r = 1.0;
    gg_sphere.color.g = 0.0;
    gg_sphere.color.b = 0.0;
    gg_sphere.color.a = 1.0; 

    pub_gg_sphere_marker_.publish(gg_sphere);

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


void Simulation_lem_ros_node::publish_dv_board_data_if_due_(){

    if (!is_due(step_number_, step_of_dv_board_data_publishing_, 0)) {
        return;
    }

    dv_interfaces::DV_board msg;
    const double R = P_.get("R");

    const double wheel_speed_left_mps = state_.omega_rr * R;  // [m/s]

    const double wheel_speed_right_mps = state_.omega_rl * R; // [m/s]
    msg.odom.velocity = (wheel_speed_left_mps + wheel_speed_right_mps) / 2.0; // średnia prędkość kół

    pub_dv_board_data_.publish(msg);



} 

void Simulation_lem_ros_node::publish_imu_if_due_()
{
    if (!is_due(step_number_, step_of_imu_publishing_, 0)) return;
    if (!has_last_imu_) return;

    const double dt_imu = P_.get("simulation_time_step") * step_of_imu_publishing_;

    // -------------------------
    // Random-walk bias update
    // -------------------------
    sim_b_smoter_ax += P_.get("acc_bias_smoter_rw")  * std::sqrt(dt_imu) * random_noise_generator_();
    sim_b_smoter_ay += P_.get("acc_bias_smoter_rw")  * std::sqrt(dt_imu) * random_noise_generator_();
    sim_b_smoter_g  += P_.get("gyro_bias_smoter_rw") * std::sqrt(dt_imu) * random_noise_generator_();

    // -------------------------
    // Compute gravity in BODY: g_B = R_BW^T * g_W
    // R_BW = R_tilt * Rz(yaw)
    // -------------------------
    const double roll  = P_.get("roll_inclination_of_world")  * M_PI / 180.0;
    const double pitch = P_.get("pitch_inclination_of_world") * M_PI / 180.0;
    const double yaw   = state_.yaw; // intrinsic yaw (Twoje)

    auto Rx = [](double r){
        double c=std::cos(r), s=std::sin(r);
        Eigen::Matrix3d R;
        R << 1,0,0,
             0,c,-s,
             0,s,c;
        return R;
    };
    auto Ry = [](double p){
        double c=std::cos(p), s=std::sin(p);
        Eigen::Matrix3d R;
        R << c,0,s,
             0,1,0,
            -s,0,c;
        return R;
    };
    auto Rz = [](double y){
        double c=std::cos(y), s=std::sin(y);
        Eigen::Matrix3d R;
        R << c,-s,0,
             s, c,0,
             0, 0,1;
        return R;
    };

    Eigen::Matrix3d R_tilt = Ry(pitch) * Rx(roll);
    Eigen::Matrix3d R_BW   = R_tilt * Rz(yaw);

    Eigen::Vector3d g_W(0.0, 0.0, -9.81);  // jeśli Z świata jest w górę
    Eigen::Vector3d g_B = R_BW.transpose() * g_W;

    // -------------------------
    // Publish IMU
    // -------------------------
    dv_interfaces::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "bolide_CoG";

    // gyro: yaw_rate + bias + noise
    imu_msg.gyro.z =
        state_.yaw_rate
        + sim_b_smoter_g
        + P_.get("gyro_noise_smoter_std") * random_noise_generator_();

    // accel: a_lin - g_B + bias + noise
    // (jeśli u Ciebie accel ma być "z grawitacją" -> dokładnie to)
    imu_msg.acc.x =
        state_.prev_ax
        - g_B.x()
        + sim_b_smoter_ax
        + P_.get("acc_noise_smoter_std") * random_noise_generator_();

    imu_msg.acc.y =
        state_.prev_ay
        - g_B.y()
        + sim_b_smoter_ay
        + P_.get("acc_noise_smoter_std") * random_noise_generator_();

    // Jeśli masz też acc.z w wiadomości i chcesz realizm:
    // imu_msg.acc.z = 0.0 - g_B.z() + sim_b_smoter_az + noise;

    pub_imu_.publish(imu_msg);
}

void Simulation_lem_ros_node::publish_steer_if_due_()
{
    
    if (!is_due(step_number_, step_of_steer_publishing_, phase_steer_publishing_)) {
        return;
    }

    dv_interfaces::Steer steer_msg;
    steer_msg.steer = state_.rack_angle;
    steer_msg.steer_dot = state_.d_rack_angle;
    pub_steer_.publish(steer_msg);
}

void Simulation_lem_ros_node::log_metric_of_ride_data_()
{
    // Jeśli nie ma ścieżki, to nic nie zapisuję
    if (metrics_log_file_path_.empty()) return;

    std::ofstream f(metrics_log_file_path_, std::ios::out);
    if (!f.is_open())
    {
        ROS_WARN_STREAM("[METRICS] Cannot open metrics file: " << metrics_log_file_path_);
        return;
    }

    const double dt = P_.get("simulation_time_step");
    const double total_time = static_cast<double>(step_number_) * dt;

    // policz procent czasu TC aktywne dopiero na końcu
    if (total_time > 1e-9)
        percetage_of_time_tc_active_ = 100.0 * time_tc_active_ / total_time;
    else
        percetage_of_time_tc_active_ = 0.0;

    if (total_time > 1e-9)
        percetage_of_time_beta_over_9_ = 100.0 * time_beta_over_9_ / total_time;
    else
        percetage_of_time_beta_over_9_ = 0.0;

    // helper do wektorów -> "a;b;c;d"
    auto join_vec = [](const std::vector<double>& v) -> std::string
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        for (size_t i = 0; i < v.size(); ++i)
        {
            if (i) oss << ";";
            oss << v[i];
        }
        return oss.str();
    };

    // CSV: proste "metric,value" - łatwe do parsowania i czytania
    f << "metric,value\n";

    f << "total_time_s," << total_time << "\n";
    f << "ey_avg_m,"      << ey_avg_   << "\n";
    f << "epsi_avg_rad,"  << epsi_avg_ << "\n";
    f << "vs_avg_mps,"    << vs_avg_   << "\n";

    f << "time_tc_active_s," << time_tc_active_ << "\n";
    f << "tc_active_percent," << percetage_of_time_tc_active_ << "\n";
    f << "time_beta_over_9deg_s," << time_beta_over_9_ << "\n";
    f << "beta_over_9deg_percent," << percetage_of_time_beta_over_9_ << "\n";

    // Top 10 listy jako jedna komórka (bezpieczne i wygodne)
    f << "ten_biggest_slip_ratio," << "\"" << join_vec(ten_biggest_slip_ratio_) << "\"\n";
    f << "ten_biggest_beta_angle," << "\"" << join_vec(ten_biggest_beta_angle_) << "\"\n";
    f << "ten_biggest_ey,"         << "\"" << join_vec(ten_biggest_ey_) << "\"\n";
    f << "ten_biggest_epsi,"       << "\"" << join_vec(ten_biggest_epsi_) << "\"\n";

    f.flush();
    f.close();

    ROS_WARN_STREAM("[METRICS] Saved ride metrics to: " << metrics_log_file_path_);
}

void Simulation_lem_ros_node::mpc_debug_callback_(const dv_interfaces::MPCDebug::ConstPtr& msg)
{
    // Avg (z kontrolera)
    epsi_avg_ = msg->epsi_avg;
    ey_avg_   = msg->ey_avg;
    vs_avg_   = msg->v_s_avg;   // albo msg->v_ref jeśli chcesz referencję

    // Current -> top10 (po module)
    update_top_abs(ten_biggest_ey_,   std::abs(msg->ey_current));
    update_top_abs(ten_biggest_epsi_, std::abs(msg->epsi_current));
}

}// namespace lem_dynamics_sim_
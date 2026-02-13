#include "wrapper.hpp"

namespace v2_control
{

using json = nlohmann::json;

// =======================
//   K O N S T R U K T O R Y
// =======================
Controller::Controller()
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Controller::Controller(ros::NodeHandle &nh, const ParamBank &param):
    stanley(param),
    pure_pursit(param),
    mpc(param)
{
    // --------------------------
    // WSTĘPNY STAN POJAZDU
    // --------------------------
    current_state = {
        0.0, // X
        0.0, // Y
        0.0, // yaw
        0.0, // delta
        0.0, // delta_dot
        0.0, // vx
        0.0, // vy
        0.0 , // yaw_rate
        0.5  // acc
    };

    mpc_state = {
        0.0, // ey
        0.0, // epsi
        0.0, // vy
        0.0, // r
        0.0, // delta
        0.0, // d_delta
        0.0  // delta_request
    };

    // --- DIAG: wczytywanie parametrów ---
    
    param_ = param;
        // ==========================================
    // PRE-KOMPUTACJA DYNAMIKI AKTUATORA (ZOH)
    // ==========================================
    double omega = param_.get("model_steer_natural_freq"); 
    double zeta  = param_.get("model_steer_damping");
    double dt    = 1.0 / param_.get("odom_frequency");

    // 1. Macierze ciągłe
    Eigen::Matrix2d A_cont;
    A_cont << 0.0, 1.0,
            -omega * omega, -2.0 * zeta * omega;

    Eigen::Vector2d B_cont(0.0, omega * omega);

    // 2. Dyskretyzacja (Matrix Exponential)
    // Ad = e^(A*dt)
    Ad_maxon = (A_cont * dt).exp(); 

    // 3. Obliczenie Bd
    // Bd = A^(-1) * (Ad - I) * B
    // Zabezpieczenie na wypadek osobliwości macierzy A (
    if (std::abs(A_cont.determinant()) > 1e-9) {
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        Bd_maxon = A_cont.inverse() * (Ad_maxon - I) * B_cont;
    } else {
        // Fallback do Eulera 
        Bd_maxon = B_cont * dt; 
    }

    // ===== SUBY =====
    path_sub = nh.subscribe("/path_planning/path", 1,
                            &Controller::pathCallback, this,
                            ros::TransportHints().tcpNoDelay());

    odom_sub = nh.subscribe("/ins/pose", 1,
                            &Controller::odometryCallback, this,
                            ros::TransportHints().tcpNoDelay());

    // ===== PUBY =====
    pub_control    = nh.advertise<dv_interfaces::Control>("/dv_board/control", 1);
    pub_geo_marker = nh.advertise<visualization_msgs::Marker>("/control/markers", 1);
    pub_ref_path   = nh.advertise<visualization_msgs::Marker>("/control/ref_path", 1);
    pub_mpc_debug = nh.advertise<dv_interfaces::MPCDebug>("/control/mpc_debug", 1);
    pub_gg_marker = nh.advertise<visualization_msgs::Marker>("/control/gg_limit_marker", 1);
} // <-- poprawnie zamknięty konstruktor






// =====================================================
//   CALLBACK PATH
// =====================================================
void Controller::pathCallback(const dv_interfaces::Path &msg)
{
    const auto &path = msg.path.poses;
 

    if(path.size() < 2)
    {
        ROS_WARN_STREAM("[Path Selection] Received path with only one point: " << path.size());
        return;
    }
    if(path.size()>=2)
    {
        if(!first_path_received)
        {
            first_path_received = true;
            ROS_INFO_STREAM("[Path Selection] First valid path received with size: " << path.size());
        }
        bool prev_last_point_valid_ = true;
        prev_last_point_.x = path[path.size()-1].position.x;
        prev_last_point_.y = path[path.size()-1].position.y;
        

    }


    X_last_from_pp.resize(path.size());
    Y_last_from_pp.resize(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        X_last_from_pp(i) = path[i].position.x;
        Y_last_from_pp(i) = path[i].position.y;
    }
    // if (path.size() > 90)
    // {
    //     ROS_WARN_STREAM("[Path Selection] Received path with very large size: " << path.size());
    // }

    // Path.msg contains only PoseArray + flag. There is no separate X/Y arrays.
    // We interpret full_path_enabled as: msg.path contains the full/global path.
    all_path_recived = msg.full_path_enabled;
}



// =====================================================
//   CALLBACK ODOMETRII
// =====================================================
void Controller::odometryCallback(const nav_msgs::Odometry &msg)
{
    ros::Time t_start = ros::Time::now();
    //std::cout << " weszłem do mapowania stanu" << std::endl;
    getCurrentState(msg);
   // std::cout << " wyszedłem z mapowania stanu" << std::endl;


    geo_control_return control_output;  

    int rozmiar_przed_procesowaniem = X_last_from_pp.size();

    // ROS_WARN_STREAM("[Path Processing] Path size before processing: "
    //                   << rozmiar_przed_procesowaniem );

    // If full_path_enabled==true, treat X_last_from_pp/Y_last_from_pp as already-global/full.
    // path_process_for_control() will still crop/transform as needed.
    
    ref_path = path_process_for_control(param_, X_last_from_pp, Y_last_from_pp,current_state,  prev_last_point_, prev_last_point_valid_, all_path_recived);
    all_path_recived = ref_path.all_path_eligible; // if path processing decided path is not eligible, reset flag
    int rozmiar_po_procesowaniu = ref_path.X_ref.size();
    path_yaw_ = ref_path.yaw_vehicle_projection;
    ey_ = ref_path.normal_distance_to_path;

    //pure_pursit.setTrack(ref_path.X_ref, ref_path.Y_ref);
    // if(rozmiar_przed_procesowaniem > rozmiar_po_procesowaniu){
    //     ROS_WARN_STREAM("[Path Processing] Processor reduced path size from "
    // //                     << rozmiar_przed_procesowaniem << " to "
    // //                     << rozmiar_po_procesowaniu << " after processing.");
    //  ROS_WARN_STREAM("[Path Processing] Path size after processing: "
    //                     << rozmiar_po_procesowaniu );
    // }


    eligible_path = ref_path.geo_eligible; // check if path is eligible for control
    if (eligible_path)
    {

        // gałąź sterowanie MPC
        if(  all_path_recived &&  current_state.vx> 2.0)
        //if(false)
        {
            convert_state_to_mpc_state();
            MPC_Return mpc_return = mpc.solve(mpc_state,ref_path.curvature,ref_path.velocity_ref ,ref_path.acceleration_ref,current_state.vx);
            //std::cout<< "Planner velocity ref: " << ref_path.velocity_ref(0) << std::endl;
            if (mpc_return.success)
            {
                last_ddelta_opt_from_mpc = mpc_return.ddelta_opt;
                last_mtv_opt_from_mpc = 0.0;
                next_target_yaw_rate_from_mpc = mpc_return.next_yaw_rate;

                double d_delta_request     = last_ddelta_opt_from_mpc / param_.get("odom_frequency"); // przekształcenie z mpc pochodnej do zmiany na krok
                double delta_request     = mpc_state.delta_request + d_delta_request;
                delta_request       = std::clamp(delta_request, param_.get("min_delta"), param_.get("max_delta"));

                mpc_state.delta_request = delta_request;
                double aceleration_request = ref_path.acceleration_ref(0);
                const double torque_request = acc_to_throttle_percentage(aceleration_request);
                publishControlCommand(delta_request,torque_request, 0.0);
                publishReferencePath(ref_path.X_ref, ref_path.Y_ref);
                ZOH_for_steering();
            }
            // gałąź awaryjna – MPC się zepsuł
            else {
                ROS_WARN_STREAM("[MPC] Solver failed. Switching to geometric controller.");
                solver_failed_ = true;
                stanley.setTrack(ref_path.X_ref, ref_path.Y_ref);
                convert_state_to_mpc_state();
                geometricControl();
                ZOH_for_steering();
            }
        }
        // gałąź sterowanie geometryczne
        else {
            stanley.setTrack(ref_path.X_ref, ref_path.Y_ref);
            convert_state_to_mpc_state();
            geometricControl();
            ZOH_for_steering();  
        }

    }


    // gałąź  – brak ścieżki , warn na konsole
    else {
        ROS_WARN_STREAM("[Control] No eligible path.");
        ZOH_for_steering();
    }


    publishGGLimitMarker();

    ros::Time t_end = ros::Time::now();
    double dt_ms = (t_end - t_start).toSec() * 1000.0;
    double dt_us = (t_end - t_start).toSec() * 1e6;

      ROS_INFO_STREAM("[Callback] odometryCallback duration: "
                    << dt_ms << " ms " );
}


// =====================================================
//   GEOMETRIC CONTROL
// =====================================================
void Controller::geometricControl()
{
    geo_control_return control_output;

    if (static_cast<int>(param_.get("using_stanley")))
    {
        control_output = stanley.StanleyControl(current_state);
    }
    else {
        control_output = pure_pursit.PurePursuitControl(current_state);
    }

    control_output.steering_angle =
        std::clamp(control_output.steering_angle, param_.get("min_delta"), param_.get("max_delta"));

    double aceleration_request = ref_path.acceleration_ref(0);
    const double torque_request = acc_to_throttle_percentage(aceleration_request);

    publishControlCommand(control_output.steering_angle,torque_request,0.0);
    publishLookaheadMarker(control_output);
    publishReferencePath(ref_path.X_ref, ref_path.Y_ref);

    mpc_state.delta_request = control_output.steering_angle;
    ZOH_for_steering();
}



// =====================================================
//   PUBLISH CONTROL
// =====================================================
void Controller::publishControlCommand(double steering_angle,double torque_request, double mtv)
{
    dv_interfaces::Control controlMsg;

    if(!all_path_recived || current_state.vx < 1.0){

    controlMsg.movement = static_cast<float>(param_.get("v_target"));
    controlMsg.steeringAngle_rad = static_cast<float>(
        std::clamp(steering_angle, param_.get("min_delta"), param_.get("max_delta"))
    );
    controlMsg.finished = false;

    controlMsg.move_type = dv_interfaces::Control::SPEED_KMH;
    controlMsg.serviceBrake = 0;
    controlMsg.current_speed = 0;
    controlMsg.fx_target = 0.0;
    controlMsg.mz_target = 0.0;

    pub_control.publish(controlMsg);

    }
    else {
       controlMsg.movement = static_cast<float>(torque_request);
        controlMsg.steeringAngle_rad = static_cast<float>(
            std::clamp(steering_angle, param_.get("min_delta"), param_.get("max_delta"))
        );
  
        controlMsg.finished = false;
    
        controlMsg.move_type = dv_interfaces::Control::TORQUE_PERCENTAGE;
        controlMsg.serviceBrake = 0;
        controlMsg.fx_target = torque_request/100*param_.get("model_max_motor_torque")/param_.get("model_wheel_radius");         
        controlMsg.mz_target =  mtv;

        controlMsg.current_speed = static_cast<float>(current_state.vx);
    
        pub_control.publish(controlMsg);
    }

}



// =====================================================
//   STAN POJAZDU – ODOMETRIA
// =====================================================
void Controller::getCurrentState(const nav_msgs::Odometry &msg)
{
    current_state.X = msg.pose.pose.position.x;
    current_state.Y = msg.pose.pose.position.y;

    tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    );

    double roll, pitch, phi;
    tf2::Matrix3x3(q).getRPY(roll, pitch, phi);

    double yaw = phi;
    unwrap_angle(yaw);
    current_state.yaw = yaw;

    // prędkości z Odom zwykle są w układzie base_link (body), ale logujemy surowe i policzone
    const double vx_msg = msg.twist.twist.linear.x;
    const double vy_msg = msg.twist.twist.linear.y;

    // jeśli vx_msg/vy_msg są GLOBAL i chcesz body -> zostaw jak było
    // (u Ciebie: global -> body rotacją -yaw)
    current_state.vx =  vx_msg * std::cos(yaw) + vy_msg * std::sin(yaw);
    current_state.vy = -vx_msg * std::sin(yaw) + vy_msg * std::cos(yaw);

    current_state.yaw_rate = msg.twist.twist.angular.z;

    // ROS_INFO_STREAM_THROTTLE(
    //     0.2,
    //     "[STATE] X=" << current_state.X
    //     << " Y=" << current_state.Y
    //     << " yaw=" << current_state.yaw
    //     << " | msg_v=(" << vx_msg << "," << vy_msg << ")"
    //     << " | body_v=(" << current_state.vx << "," << current_state.vy << ")"
    //     << " r=" << current_state.yaw_rate
    // );
}



// =====================================================
//   LOOKAHEAD MARKER
// =====================================================
void Controller::publishLookaheadMarker(const geo_control_return &control_output)
{
    visualization_msgs::Marker lookahead_marker;
    lookahead_marker.header.frame_id = "map";
    lookahead_marker.header.stamp    = ros::Time(0);
    lookahead_marker.ns  = "lookahead_point";
    lookahead_marker.id  = 1;
    lookahead_marker.type = visualization_msgs::Marker::CUBE;
    lookahead_marker.action = visualization_msgs::Marker::ADD;

    lookahead_marker.pose.position.x = control_output.look_ahead_point.x;
    lookahead_marker.pose.position.y = control_output.look_ahead_point.y;
    lookahead_marker.pose.position.z = 0.05;
    lookahead_marker.pose.orientation.w = 1.0;

    lookahead_marker.scale.x = 0.2;
    lookahead_marker.scale.y = 0.2;
    lookahead_marker.scale.z = 0.01;

    lookahead_marker.color.a = 1.0;
    lookahead_marker.color.r = 1.0;
    lookahead_marker.color.g = 0.0;
    lookahead_marker.color.b = 0.0;

    pub_geo_marker.publish(lookahead_marker);
}



// =====================================================
//   PATH POINTS – KROPKI
// =====================================================
void Controller::publishReferencePath(const Eigen::VectorXd &X,
    const Eigen::VectorXd &Y)
{
for (int i = 0; i <  X.size(); ++i)
{
visualization_msgs::Marker m;

m.header.frame_id = "map";
m.header.stamp    = ros::Time::now();

m.ns = "ref_path_points";
m.id = i + 200000;                       // unikalny ID dla każdego punktu

m.type   = visualization_msgs::Marker::SPHERE;
m.action = visualization_msgs::Marker::ADD;

m.pose.position.x = X(i);
m.pose.position.y = Y(i);
m.pose.position.z = 0.05;

m.scale.x = 0.10;
m.scale.y = 0.10;
m.scale.z = 0.10;

m.color.r = 0.0;
m.color.g = 1.0;   // zielony
m.color.b = 0.0;
m.color.a = 1.0;

m.lifetime = ros::Duration(0);  // marker jest wieczny
m.frame_locked = true;          // NIE znika przy ruchu mapy

pub_ref_path.publish(m); // <-- używamy osobnego topica
}
}


// =====================================================
//   MODEL II RZĘDU – DYSKRETYZACJA ZOH
// =====================================================
void Controller::ZOH_for_steering()
{
    double max_speed = param_.get("model_max_steering_angle_rate"); // rad/s
    double dt = 1.0 / param_.get("odom_frequency"); 

    double old_delta   = mpc_state.delta;
    double old_d_delta = mpc_state.d_delta;

    // 1. Idealne przewidywanie ZOH (zakładamy brak limitów)
    double exact_delta   = Ad_maxon(0,0) * old_delta + Ad_maxon(0,1) * old_d_delta + Bd_maxon(0) * mpc_state.delta_request;
    double exact_d_delta = Ad_maxon(1,0) * old_delta + Ad_maxon(1,1) * old_d_delta + Bd_maxon(1) * mpc_state.delta_request;

    // 2. Sprawdzamy, czy w ogóle przekroczyliśmy limit w tym kroku
    if (std::abs(exact_d_delta) <= max_speed) 
    {
        // SYTUACJA A: Cały krok jest w strefie liniowej. 
        mpc_state.delta   = exact_delta;
        mpc_state.d_delta = exact_d_delta;
    } 
    else 
    {
        // SYTUACJA B: Przekroczyliśmy limit! Musimy znaleźć moment uderzenia w ścianę.
        
        // Znak kierunku (1.0 w lewo, -1.0 w prawo)
        double dir = (exact_d_delta > 0) ? 1.0 : -1.0; 

        // Jeśli zaczęliśmy krok już na limicie (lub powyżej), cały krok to "dach trapezu"
        if (std::abs(old_d_delta) >= max_speed) 
        {
            mpc_state.d_delta = dir * max_speed;
            mpc_state.delta   = old_delta + mpc_state.d_delta * dt;
        }
        else 
        {
            // SYTUACJA C: Zaczęliśmy poniżej limitu, ale go przebiliśmy (Uderzenie w trakcie kroku).
            // Liczymy JAKĄ CZĘŚĆ kroku zajęło nam dobicie do limitu (tzw. "fraction" od 0.0 do 1.0)
            double fraction = (max_speed - std::abs(old_d_delta)) / (std::abs(exact_d_delta) - std::abs(old_d_delta));
            
            double t_accel = fraction * dt;         // Czas rozpędzania (zbocze)
            double t_const = (1.0 - fraction) * dt; // Czas jazdy na limicie (dach)

            // Całka z prędkości to droga. Liczymy pole pod "uciętym" wykresem:
            // 1. Pole trapezu (rozpędzanie od starej prędkości do max_speed)
            double dist_accel = dir * ( (std::abs(old_d_delta) + max_speed) / 2.0 ) * t_accel;
            // 2. Pole prostokąta (jazda ze stałą prędkością max_speed)
            double dist_const = dir * max_speed * t_const;

            // Zapisujemy idealnie wyliczoną pozycję i obciętą prędkość końcową
            mpc_state.delta   = old_delta + dist_accel + dist_const;
            mpc_state.d_delta = dir * max_speed;
        }
    }
    mpc_state.delta = std::clamp(mpc_state.delta, param_.get("min_delta"), param_.get("max_delta"));
}

double Controller::acc_to_throttle_percentage(double a_desired)
{
    //std::cout<< "ACC:"<<a_desired<<std::endl;
    double vehicle_mass = param_.get("model_m");
    double wheel_radius = param_.get("model_wheel_radius");
    double max_motor_torque =  param_.get("model_max_motor_torque");
    const double drag_force = param_.get("model_Cd") * current_state.vx * current_state.vx + 
                              param_.get("model_rolling_resistance_coeff") * vehicle_mass * 9.81;
   // a_desired += drag_force / vehicle_mass; // uwzględnienie oporów ruchu

    double F_required = vehicle_mass * a_desired - vehicle_mass*current_state.yaw_rate*current_state.vy  ; // Siła wymagana do osiągnięcia przyspieszenia
    double torque_required = F_required * wheel_radius; // Moment obrotowy wymagany na kołach

    // Przekształcenie momentu obrotowego na sygnał sterujący przepustnicą (-1.0 - 1.0) -> percentage of all torque
    double throttle_command = torque_required / max_motor_torque*100.0;

    throttle_command = std::clamp(throttle_command, -100.0, 100.0);
    current_state.acc = a_desired;

    return throttle_command;
}



static inline double wrap_to_pi(double a)
{
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

void Controller::convert_state_to_mpc_state()
{
    const int n = ref_path.X_ref.size();
    if (n < 2) return;

    // =========================
    // 1) closest-point w oknie
    // =========================
    int idx_closest = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < n ; ++i) {
        const double dx = current_state.X - ref_path.X_ref(i);
        const double dy = current_state.Y - ref_path.Y_ref(i);
        const double d2 = dx*dx + dy*dy;
        if (d2 < min_dist) {
            min_dist = d2;
            idx_closest = i;
        }
    }

    // =========================
    // 2) wybór segmentu: (i-1,i) vs (i,i+1)
    // =========================
    auto proj_dist_sq_on_segment = [&](int i0, int i1, double& t_out, double& px, double& py) -> double
    {
        const double x0 = ref_path.X_ref(i0), y0 = ref_path.Y_ref(i0);
        const double x1 = ref_path.X_ref(i1), y1 = ref_path.Y_ref(i1);

        const double sx = x1 - x0;
        const double sy = y1 - y0;
        const double seg_len_sq = sx*sx + sy*sy;

        if (seg_len_sq < 1e-12) {
            t_out = 0.0; px = x0; py = y0;
            const double dx = current_state.X - px;
            const double dy = current_state.Y - py;
            return dx*dx + dy*dy;
        }

        const double cx = current_state.X - x0;
        const double cy = current_state.Y - y0;

        double t = (cx*sx + cy*sy) / seg_len_sq;
        t = std::clamp(t, 0.0, 1.0);   // <<< KLUCZOWE
        t_out = t;

        px = x0 + t*sx;
        py = y0 + t*sy;

        const double dx = current_state.X - px;
        const double dy = current_state.Y - py;
        return dx*dx + dy*dy;
    };


    int best_i0 = std::clamp(idx_closest, 0, n-2);
    int best_i1 = best_i0 + 1;

    double tA=0, pxA=0, pyA=0;
    double dA = std::numeric_limits<double>::infinity();
    if (idx_closest >= 1) dA = proj_dist_sq_on_segment(idx_closest-1, idx_closest, tA, pxA, pyA);


    double tB=0, pxB=0, pyB=0;
    double dB = std::numeric_limits<double>::infinity();
    if (idx_closest <= n-2) dB = proj_dist_sq_on_segment(idx_closest, idx_closest+1, tB, pxB, pyB);


    double proj_X, proj_Y, t;
    if (dA < dB) {
        best_i0 = idx_closest - 1; best_i1 = idx_closest;
        proj_X = pxA; proj_Y = pyA; t = tA;
    } else {
        best_i0 = idx_closest; best_i1 = idx_closest + 1;
        proj_X = pxB; proj_Y = pyB; t = tB;
    }


   
    // =========================
    // 5) Zapis do mpc_state
    // =========================
    static bool   have_prev = false;
    static double ey_prev   = 0.0;
    static double epsi_prev = 0.0;
    static int    seg_i0_prev = -1;
    static int    seg_i1_prev = -1;

    mpc_state.ey   = ey_;
    double epsi = wrap_to_pi(current_state.yaw - path_yaw_);
    mpc_state.epsi = epsi;
    mpc_state.r    = current_state.yaw_rate;
    mpc_state.vy   = current_state.vy;

  
    ey_count++;
    ey_sum += std::abs(ey_);
    epsi_sum += std::abs(epsi);
    // update prev
    have_prev   = true;
    ey_prev     = ey_;
    epsi_prev   = epsi;
    seg_i0_prev = best_i0;
    seg_i1_prev = best_i1;
    dv_interfaces::MPCDebug mpc_debug_msg;
    mpc_debug_msg.ey_avg = ey_sum / ey_count;
    mpc_debug_msg.epsi_avg = epsi_sum / ey_count;
    mpc_debug_msg.ey_current = mpc_state.ey;
    mpc_debug_msg.epsi_current = mpc_state.epsi;
    mpc_debug_msg.v_ref = ref_path.velocity_ref(best_i1);
    mpc_debug_msg.kappa_ref = ref_path.curvature(best_i0);
    mpc_debug_msg.mpc_yaw_rate_from_curvature = ref_path.curvature(best_i0)*mpc_debug_msg.v_ref;
    if(std::abs(ref_path.curvature(best_i0))>1e-4){
    mpc_debug_msg.R_ref = 1.0/ref_path.curvature(best_i0);
    }
    else {
    mpc_debug_msg.R_ref = 1e4; // duży promień przy małej krzywiźnie
    }
    mpc_debug_msg.alat_ref = ref_path.velocity_ref(best_i1)*ref_path.velocity_ref(best_i1)*(ref_path.curvature(best_i0));
    mpc_debug_msg.v_s_current = current_state.vx*std::cos(mpc_state.epsi) - current_state.vy*std::sin(mpc_state.epsi);
    v_s_sum += current_state.vx*std::cos(mpc_state.epsi) - current_state.vy*std::sin(mpc_state.epsi);
    mpc_debug_msg.v_s_avg = v_s_sum/ey_count;
    mpc_debug_msg.next_yaw_rate_target = next_target_yaw_rate_from_mpc;
    //std::cout << "next_target_yaw_rate_from_mpc: " << next_target_yaw_rate_from_mpc << std::endl;
    //std::cout<< "ref lat" <<  mpc_debug_msg.alat_ref <<std::endl;
    mpc_debug_msg.ax_target = ref_path.acceleration_ref(best_i0);


    mpc_debug_msg.mpc_steer_angle = mpc_state.delta*180.0/M_PI;
    mpc_debug_msg.mpc_steer_rate = mpc_state.d_delta*180.0/M_PI;
    mpc_debug_msg.solver_failed = solver_failed_; // do uzupełnienia po MPC
    pub_mpc_debug.publish(mpc_debug_msg);
    if(std::abs(mpc_debug_msg.R_ref )< 4.5){
        ROS_WARN_STREAM_THROTTLE(1.0,"[MPC] Small turning radius R_ref="<< mpc_debug_msg.R_ref<<" m");
    }
}



void Controller::publishGGLimitMarker()
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "gg_dashboard"; // Układ odniesienia: środek auta
    msg.header.stamp    = ros::Time::now();
    msg.ns = "gg_envelope";
    msg.id = 0;
    msg.type   = visualization_msgs::Marker::LINE_STRIP; // Typ: połączone punkty
    msg.action = visualization_msgs::Marker::ADD;

    // Wygląd linii (grubość i kolor szary)
    msg.scale.x = 1.03; 
    msg.color.r = 0.5;  
    msg.color.g = 0.5; 
    msg.color.b = 0.5; 
    msg.color.a = 1.0;

    // Pobranie limitów z parametrów
    double max_ax = param_.get("vel_planner_max_accel"); 
    double max_ay = param_.get("vel_planner_max_corrnering_accel");

    // Generowanie pełnego okręgu (lub elipsy) ze 100 punktów
    for (int i = 0; i <= 100; ++i) {
        double theta = (i / 100.0) * 2.0 * M_PI;
        geometry_msgs::Point p;
        // Foxglove oś X = przyspieszenie boczne (y auta)
        // Foxglove oś Y = przyspieszenie wzdłużne (x auta)
        p.x = max_ay * std::cos(theta); 
        p.y = max_ax * std::sin(theta); 
        p.z = 0.0;
        msg.points.push_back(p);
    }

    pub_gg_marker.publish(msg);
}
        
} // namespace v2_control

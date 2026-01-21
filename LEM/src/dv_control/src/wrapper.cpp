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
        ROS_WARN_STREAM("[Path Selection] Refreance path is not updated.");
        return;
    }

    X_last_from_pp.resize(path.size());
    Y_last_from_pp.resize(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        X_last_from_pp(i) = path[i].position.x;
        Y_last_from_pp(i) = path[i].position.y;
    }
    if (path.size() > 90)
    {
        ROS_WARN_STREAM("[Path Selection] Received path with very large size: " << path.size());
    }

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
    getCurrentState(msg);

    geo_control_return control_output;  

    int rozmiar_przed_procesowaniem = X_last_from_pp.size();

    // ROS_WARN_STREAM("[Path Processing] Path size before processing: "
    //                   << rozmiar_przed_procesowaniem );

    // If full_path_enabled==true, treat X_last_from_pp/Y_last_from_pp as already-global/full.
    // path_process_for_control() will still crop/transform as needed.
    ref_path = path_process_for_control(param_, X_last_from_pp, Y_last_from_pp,current_state, all_path_recived);
    int rozmiar_po_procesowaniu = ref_path.X_ref.size();
    path_yaw_ = ref_path.yaw0;
    //pure_pursit.setTrack(ref_path.X_ref, ref_path.Y_ref);


    // if(rozmiar_przed_procesowaniem > rozmiar_po_procesowaniu){
    //     ROS_WARN_STREAM("[Path Processing] Processor reduced path size from "
    // //                     << rozmiar_przed_procesowaniem << " to "
    // //                     << rozmiar_po_procesowaniu << " after processing.");
    //  ROS_WARN_STREAM("[Path Processing] Path size after processing: "
    //                     << rozmiar_po_procesowaniu );
    // }
    eligible_path = ref_path.mpc_eligible || ref_path.geo_eligible;
    if (eligible_path)
    {
        //if (ref_path.mpc_eligible)
        if(ref_path.mpc_eligible && current_state.vx> 1.0)
        //if(false)
        {
            convert_state_to_mpc_state();
            MPC_Return mpc_return = mpc.solve(mpc_state,ref_path.curvature,ref_path.velocity_ref ,ref_path.acceleration_ref,current_state.vx);
            //std::cout<< "Planner velocity ref: " << ref_path.velocity_ref(0) << std::endl;
            if (mpc_return.success)
            {
                last_u_opt_from_mpc = mpc_return.u_opt;
                double d_delta_request     = last_u_opt_from_mpc / param_.get("odom_frequency");
                double delta_request     = mpc_state.delta_request + d_delta_request;
                delta_request       = std::clamp(delta_request, param_.get("min_delta"), param_.get("max_delta"));

                mpc_state.delta_request = delta_request;
                double aceleration_request = ref_path.acceleration_ref(0);
                const double torque_request = acc_to_throttle_percentage(aceleration_request);
                publishControlCommand(delta_request,torque_request);
                publishReferencePath(ref_path.X_ref, ref_path.Y_ref);
                ZOH_for_steering();
                //ROS_WARN_STREAM("[MPC]");
            }
            else {
                ROS_WARN_STREAM("[MPC] Solver failed. Switching to geometric controller.");
                stanley.setTrack(ref_path.X_ref, ref_path.Y_ref);
                convert_state_to_mpc_state();
                geometricControl();
                ZOH_for_steering();
                ROS_WARN_STREAM("[GEOMETRIC AFTR MPC FAIL]");
            }
        }
        else {
            stanley.setTrack(ref_path.X_ref, ref_path.Y_ref);
            convert_state_to_mpc_state();
            geometricControl();
            ZOH_for_steering();
            //ROS_WARN_STREAM("[GEOMETRIC]");
            
        }
    }
        
    else {
        ROS_WARN_STREAM("[Control] No eligible path.");
        ZOH_for_steering();
    }

    ros::Time t_end = ros::Time::now();
    double dt_ms = (t_end - t_start).toSec() * 1000.0;
    double dt_us = (t_end - t_start).toSec() * 1e6;

    //   ROS_INFO_STREAM("[Callback] odometryCallback duration: "
    //                 << dt_ms << " ms " );
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

    publishControlCommand(control_output.steering_angle,torque_request);
    publishLookaheadMarker(control_output);
    publishReferencePath(ref_path.X_ref, ref_path.Y_ref);

    mpc_state.delta_request = control_output.steering_angle;
    ZOH_for_steering();
}



// =====================================================
//   PUBLISH CONTROL
// =====================================================
void Controller::publishControlCommand(double steering_angle,double torque_request)
{
    dv_interfaces::Control controlMsg;

    if(!all_path_recived){

    controlMsg.movement = static_cast<float>(param_.get("v_target"));
    controlMsg.steeringAngle_rad = static_cast<float>(
        std::clamp(steering_angle, param_.get("min_delta"), param_.get("max_delta"))
    );
    controlMsg.finished = false;

    controlMsg.move_type = dv_interfaces::Control::SPEED_KMH;
    controlMsg.serviceBrake = 0;
    controlMsg.current_speed = 0;

    pub_control.publish(controlMsg);

    }
    else {
        controlMsg.movement = static_cast<float>(torque_request*100);
        controlMsg.steeringAngle_rad = static_cast<float>(
            std::clamp(steering_angle, param_.get("min_delta"), param_.get("max_delta"))
        );
        controlMsg.finished = false;
    
        controlMsg.move_type = dv_interfaces::Control::TORQUE_PERCENTAGE;
        controlMsg.serviceBrake = 0;

        controlMsg.current_speed = static_cast<float>(current_state.vx);
    
        pub_control.publish(controlMsg);
    }

    // }

    // controlMsg.movement = static_cast<float>(param_.get("v_target"));
    // controlMsg.steeringAngle_rad = static_cast<float>(
    //      std::clamp(steering_angle, param_.get("min_delta"), param_.get("max_delta"))
    //  );
    // controlMsg.finished = false;

    // controlMsg.move_type = dv_interfaces::Control::SPEED_KMH;
    // controlMsg.serviceBrake = 0;

    // pub_control.publish(controlMsg);


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
   
    double old_delta   = mpc_state.delta;
    double old_d_delta = mpc_state.d_delta;

    mpc_state.delta   = Ad_maxon(0,0) * old_delta + Ad_maxon(0,1) * old_d_delta + Bd_maxon(0) * mpc_state.delta_request;
    mpc_state.d_delta = Ad_maxon(1,0) * old_delta + Ad_maxon(1,1) * old_d_delta + Bd_maxon(1) * mpc_state.delta_request;
}

double Controller::acc_to_throttle_percentage(double a_desired)
{
    //std::cout<< "ACC:"<<a_desired<<std::endl;
    double vehicle_mass = param_.get("model_m");
    double wheel_radius = param_.get("model_wheel_radius");
    double max_motor_torque = param_.get("model_max_motor_torque");

    double F_required = vehicle_mass * a_desired - vehicle_mass*current_state.yaw_rate*current_state.vy; // Siła wymagana do osiągnięcia przyspieszenia
    double torque_required = F_required * wheel_radius; // Moment obrotowy wymagany na kołach

    // Przekształcenie momentu obrotowego na sygnał sterujący przepustnicą (-1.0 - 1.0)
    double throttle_command = torque_required / max_motor_torque;

    throttle_command = std::clamp(throttle_command, -1.0, 1.0);
    current_state.acc = a_desired;
    //std::cout<< "THROTTLE:"<<throttle_command<<std::endl;

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
    static int last_closest = 0;
    const int W = 60; // dostosuj (gęstość punktów / prędkość / dt)

    int i_min = std::max(0, last_closest - W);
    int i_max = std::min(n - 1, last_closest + W);

    int idx_closest = i_min;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = i_min; i <= i_max; ++i) {
        const double dx = current_state.X - ref_path.X_ref(i);
        const double dy = current_state.Y - ref_path.Y_ref(i);
        const double d2 = dx*dx + dy*dy;
        if (d2 < min_dist) {
            min_dist = d2;
            idx_closest = i;
        }
    }
    last_closest = idx_closest;

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
    // 3) path_yaw, ey, epsi
    // =========================
   

    const double dX_proj = current_state.X - proj_X;
    const double dY_proj = current_state.Y - proj_Y;

    const double sin_psi = std::sin(path_yaw_);
    const double cos_psi = std::cos(path_yaw_);

    const double ey   = dY_proj * cos_psi - dX_proj * sin_psi;
    const double epsi = wrap_to_pi(current_state.yaw - path_yaw_);

    // =========================
    // 4) Δey, Δepsi (vs poprzednia iteracja) + logika skoków
    // =========================
    static bool   have_prev = false;
    static double ey_prev   = 0.0;
    static double epsi_prev = 0.0;
    static int    seg_i0_prev = -1;
    static int    seg_i1_prev = -1;

    double d_ey = 0.0;
    double d_epsi = 0.0;

    if (have_prev) {
        d_ey   = ey - ey_prev;
        d_epsi = wrap_to_pi(epsi - epsi_prev); // ważne przy przejściu przez +/-pi
    }

    // progi “skoku” (dostosuj do dt i typowego noise)
    const double EY_JUMP_TH   = 0.30;  // [m]
    const double EPSI_JUMP_TH = 0.15;  // [rad]

    const bool seg_changed = (seg_i0_prev != best_i0) || (seg_i1_prev != best_i1);
    const bool jump = have_prev && (std::abs(d_ey) > EY_JUMP_TH || std::abs(d_epsi) > EPSI_JUMP_TH);

    // =========================
    // 5) Zapis do mpc_state
    // =========================
    mpc_state.ey   = ey;
    mpc_state.epsi = epsi;
    mpc_state.r    = current_state.yaw_rate;
    mpc_state.vy   = current_state.vy;

    // =========================
    // 6) LOGI (throttle + event-based)
    // =========================
    // 1) normal debug rzadko
    // ROS_INFO_STREAM_THROTTLE(0.2,
    //     "[MPC PROJ] idx_closest=" << idx_closest
    //     << " seg=(" << best_i0 << "," << best_i1 << ")"
    //     << " t=" << t
    //     << " path_yaw=" << path_yaw_
    //     << " ey=" << ey << " epsi=" << epsi
    //     << " d_ey=" << d_ey << " d_epsi=" << d_epsi
    //     << " seg_changed=" << (seg_changed ? 1 : 0)
    // );

    // 2) gdy wykryję skok — WARN natychmiast
    // if (jump) {
    //     ROS_WARN_STREAM(
    //         "[MPC JUMP] d_ey=" << d_ey << " (ey=" << ey_prev << "->" << ey << ")"
    //         << " | d_epsi=" << d_epsi << " (epsi=" << epsi_prev << "->" << epsi << ")"
    //         << " | idx_closest=" << idx_closest
    //         << " | seg=(" << best_i0 << "," << best_i1 << ")"
    //         << " | seg_changed=" << (seg_changed ? 1 : 0)
    //         << " | car_xy=(" << current_state.X << "," << current_state.Y << ")"
    //         << " | proj_xy=(" << proj_X << "," << proj_Y << ")"
    //     );
    // }
    ey_count++;
    ey_sum += std::abs(ey);
    epsi_sum += std::abs(epsi);
    // update prev
    have_prev   = true;
    ey_prev     = ey;
    epsi_prev   = epsi;
    seg_i0_prev = best_i0;
    seg_i1_prev = best_i1;
    dv_interfaces::MPCDebug mpc_debug_msg;
    mpc_debug_msg.ey_avg = ey_sum / ey_count;
    mpc_debug_msg.epsi_avg = epsi_sum / ey_count;
    mpc_debug_msg.ey_current = mpc_state.ey;
    mpc_debug_msg.epsi_current = mpc_state.epsi;
    mpc_debug_msg.v_ref = ref_path.velocity_ref(1);
    mpc_debug_msg.kappa_ref = ref_path.curvature(0);
    if(std::abs(ref_path.curvature(0))>1e-4){
    mpc_debug_msg.R_ref = 1.0/ref_path.curvature(0);
    }
    else {
    mpc_debug_msg.R_ref = 1e4; // duży promień przy małej krzywiźnie
    }
    mpc_debug_msg.alat_ref = ref_path.velocity_ref(0)*ref_path.velocity_ref(0)*std::abs(ref_path.curvature(0));
    mpc_debug_msg.v_s_current = current_state.vx*std::cos(mpc_state.epsi) - current_state.vy*std::sin(mpc_state.epsi);
    v_s_sum += current_state.vx*std::cos(mpc_state.epsi) - current_state.vy*std::sin(mpc_state.epsi);
    mpc_debug_msg.v_s_avg = v_s_sum/ey_count;
    pub_mpc_debug.publish(mpc_debug_msg);
    if(std::abs(mpc_debug_msg.R_ref )< 4.5){
        ROS_WARN_STREAM_THROTTLE(1.0,"[MPC] Small turning radius R_ref="<< mpc_debug_msg.R_ref<<" m");
    }
}
        
} // namespace v2_control

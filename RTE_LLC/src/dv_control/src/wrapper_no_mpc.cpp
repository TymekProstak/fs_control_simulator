#include "wrapper_no_mpc.hpp"

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
        0.0  // yaw_rate
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
    // ROS_INFO("Received new path.");

    const auto &path = msg.path.poses;
   

    if(path.size() < 2)
    {
        ROS_WARN_STREAM("[Path Selection] Received path with too few points: " << path.size());
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

    // for parity with wrapper.cpp; currently the no-mpc path processor ignores this flag
    first_path_received = msg.full_path_enabled;
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

    ref_path = path_process_for_control(param_, X_last_from_pp, Y_last_from_pp, current_state.X, current_state.Y);
    int rozmiar_po_procesowaniu = ref_path.X_ref.size();
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
            stanley.setTrack(ref_path.X_ref, ref_path.Y_ref);
            convert_state_to_mpc_state();
            geometricControl();
            ZOH_for_steering();
            //ROS_WARN_STREAM("[GEOMETRIC]");
    }
        
    else {
        ROS_WARN_STREAM("[Control] No eligible path.");
        ZOH_for_steering();
    }

    ros::Time t_end = ros::Time::now();
    double dt_ms = (t_end - t_start).toSec() * 1000.0;
    double dt_us = (t_end - t_start).toSec() * 1e6;

    //  ROS_INFO_STREAM("[Callback] odometryCallback duration: "
    //                << dt_ms << " ms " );
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

    publishControlCommand(control_output.steering_angle);
    publishLookaheadMarker(control_output);
    publishReferencePath(ref_path.X_ref, ref_path.Y_ref);

    mpc_state.delta_request = control_output.steering_angle;
    ZOH_for_steering();
}



// =====================================================
//   PUBLISH CONTROL
// =====================================================
void Controller::publishControlCommand(double steering_angle)
{
    dv_interfaces::Control controlMsg;

    controlMsg.movement = static_cast<float>(param_.get("v_target"));
    controlMsg.steeringAngle_rad = static_cast<float>(
        std::clamp(steering_angle, param_.get("min_delta"), param_.get("max_delta"))
    );
    controlMsg.finished = false;
    controlMsg.move_type = dv_interfaces::Control::SPEED_KMH;
    controlMsg.serviceBrake = 0;

    pub_control.publish(controlMsg);
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

    double vx_global = msg.twist.twist.linear.x;
    double vy_global = msg.twist.twist.linear.y;

    current_state.vx =  vx_global * cos(phi) + vy_global * sin(phi);
    current_state.vy = -vx_global * sin(phi) + vy_global * cos(phi);
    current_state.yaw_rate = msg.twist.twist.angular.z;
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



// =====================================================
//   KONWERSJA STANU DO STANU MPC
// =====================================================
void Controller::convert_state_to_mpc_state()
{
    int n_path_points = ref_path.X_ref.size();
    if (n_path_points < 2) return; // Zabezpieczenie przed zbyt krótką ścieżką

    // 1. Znalezienie najbliższego punktu (indeksu)
    int indx_closest = 0;
    double min_dist = std::numeric_limits<double>::max();
    for(int i = 0; i < n_path_points; i++){
        double dist = std::hypot(current_state.X - ref_path.X_ref(i),
                                 current_state.Y - ref_path.Y_ref(i));
        if(dist < min_dist){
            min_dist = dist;
            indx_closest = i;
        }
    }

    // 2. Wybór segmentu do rzutowania (pomiędzy i oraz i+1)
    // Jeśli najbliższy punkt jest ostatni, musimy cofnąć się o jeden, by mieć odcinek
    int i0 = indx_closest;
    if (i0 >= n_path_points - 1) {
        i0 = n_path_points - 2;
    }
    int i1 = i0 + 1;

    // 3. Wektory segmentu ścieżki (P0 -> P1)
    double dx_seg = ref_path.X_ref(i1) - ref_path.X_ref(i0);
    double dy_seg = ref_path.Y_ref(i1) - ref_path.Y_ref(i0);
    double seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg;

    // 4. Wektor od początku segmentu do auta (P0 -> Car)
    double dx_car = current_state.X - ref_path.X_ref(i0);
    double dy_car = current_state.Y - ref_path.Y_ref(i0);

    // 5. Rzutowanie (iloczyn skalarny) - wyznaczamy jak daleko "wzdłuż" segmentu jesteśmy
    // t to parametr odcinka (0 = początek, 1 = koniec)
    double t = (dx_car * dx_seg + dy_car * dy_seg) / seg_len_sq;
    
    // Ograniczenie t do [0,1] by pozostać na segmencie
    //t = std::clamp(t, 0.0, 1.0);
    // 6. Wyznaczenie punktu rzutu na ścieżce (punkt na linii najbliższy autu)
    double proj_X = ref_path.X_ref(i0) + t * dx_seg;
    double proj_Y = ref_path.Y_ref(i0) + t * dy_seg;

    // 7. Obliczenie kąta ścieżki (styczny do segmentu)
    double path_yaw = std::atan2(dy_seg, dx_seg);

    // 8. Obliczenie ey (błąd poprzeczny) względem punktu rzutu
    // Używamy różnicy współrzędnych od punktu rzutu
    double dX_proj = current_state.X - proj_X;
    double dY_proj = current_state.Y - proj_Y;

    double sin_psi = std::sin(path_yaw);
    double cos_psi = std::cos(path_yaw);

    // Wzór na ey (Frenet) - dodatni jeśli auto jest po lewej stronie ścieżki
    mpc_state.ey = dY_proj * cos_psi - dX_proj * sin_psi;

    // 9. Błąd orientacji
    double epsi = current_state.yaw - path_yaw;
    unwrap_angle(epsi);      
    mpc_state.epsi = epsi;

    // Reszta stanów
    mpc_state.r  = current_state.yaw_rate;
    mpc_state.vy = current_state.vy;

    // --- Logika Debug/Publishing ---
    ey_count += 1;
    ey_sum   += std::abs(mpc_state.ey);
    epsi_sum += std::abs(mpc_state.epsi);

    dv_interfaces::MPCDebug mpc_debug_msg;
    mpc_debug_msg.ey_avg = ey_sum / ey_count;
    mpc_debug_msg.epsi_avg = epsi_sum / ey_count;
    mpc_debug_msg.ey_current = mpc_state.ey;
    mpc_debug_msg.epsi_current = mpc_state.epsi;
    pub_mpc_debug.publish(mpc_debug_msg);
}
} // namespace v2_control

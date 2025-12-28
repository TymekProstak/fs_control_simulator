#include "pure_pursuit.hpp"

namespace v2_control{

Pure_Pursuit::Pure_Pursuit(){
   std::cout<<" deafult initiliazer not everything is correct"<<std::endl;
}
Pure_Pursuit::Pure_Pursuit( const ParamBank &P){
    
    // inclizacja na zero -> by nie wyrzuciło na dzień dobry głupot
    lookAheadPointInGlobalFrame = Vec2(0,0);
    
    // wczyranie parametrów
    
    param_ = P;

    y_error_in_vehicle_frame = 0;
}

void Pure_Pursuit::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y)
{
    if (X.size() != Y.size()) {
        ROS_WARN_STREAM("[Pure_Pursuit] Rozmiary wektorów X (" << X.size()
                         << ") i Y (" << Y.size()
                         << ") nie są równe. Ścieżka NIE została zaktualizowana.");
        return;   // nie zmieniam ptrack_
    }
    if(X.size() <2){
        ROS_WARN_STREAM("[Pure_Pursuit] Za mało punktów na ścieżce: " 
            << X.size() << ". Funkcja przerwana.");
            return;  // nic nie zmieniamy w stanie obiektu
    }

    ptrack_.X = X;
    ptrack_.Y = Y;
    ptrack_.n_points = static_cast<int>(X.size());

}

double Pure_Pursuit::computeSteeringAngle(const State &bolid_state) 
{   

    // asuming constant velocity of 3 m/s for now no velocity/curvature term
    double alfa = std::asin( y_error_in_vehicle_frame / param_.get("pp_look_ahead_distance") );
    return std::atan2(2.0 * param_.get("pp_l") * std::sin(alfa), param_.get("pp_look_ahead_distance"));
    
}

void Pure_Pursuit::findLookAheadPoint(const State &bolid_state)
{
    
    
     if (ptrack_.n_points < 2) {
        ROS_WARN_STREAM("[Pure_Pursuit] Za mało punktów na ścieżce: " 
            << ptrack_.n_points << ". Funkcja przerwana.");
            return;  // nic nie zmieniamy w stanie obiektu
    }

    double bolide_yaw = bolid_state.yaw;
   
    unwrap_angle(bolide_yaw);

   

    double X_rear = bolid_state.X - param_.get("pp_lr") * std::cos(bolide_yaw);
    double Y_rear = bolid_state.Y - param_.get("pp_lr") * std::sin(bolide_yaw);


    // finding the first point on the track further than look ahead distance from rear axle
    // this point will be our look ahead point
    int idx_look_ahead = 0;
    for( int idx = 0; idx < ptrack_.n_points;idx++){

        double dist_sqr = (X_rear - ptrack_.X(idx))*(X_rear - ptrack_.X(idx)) + (Y_rear - ptrack_.Y(idx))*(Y_rear - ptrack_.Y(idx));
        if(dist_sqr > param_.get("pp_look_ahead_distance") * param_.get("pp_look_ahead_distance") ) {

            break;
        }
        idx_look_ahead = idx;
    }


    // ustawienie look ahead pointu i y_error_in_vehicle_frame
    lookAheadPointInGlobalFrame.x = ptrack_.X(idx_look_ahead);

    lookAheadPointInGlobalFrame.y = ptrack_.Y(idx_look_ahead);
    // wektor z tylnej osi -> punkt look ahead
    double dx = lookAheadPointInGlobalFrame.x - X_rear;
    double dy = lookAheadPointInGlobalFrame.y - Y_rear;

    // transformacja do układu pojazdu (osobno x i y)
    double x_v =  std::cos(bolide_yaw) * dx + std::sin(bolide_yaw) * dy;
    double y_v = -std::sin(bolide_yaw) * dx + std::cos(bolide_yaw) * dy;

    y_error_in_vehicle_frame = y_v;
    


}

geo_control_return Pure_Pursuit::PurePursuitControl(const State &bolid_state) 
{
    findLookAheadPoint(bolid_state);
    double steering_angle = computeSteeringAngle(bolid_state);
    
    geo_control_return control_output;
    control_output.look_ahead_point = lookAheadPointInGlobalFrame;
    control_output.steering_angle = steering_angle;
    
    return control_output;

}

}
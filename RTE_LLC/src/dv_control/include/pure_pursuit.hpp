#pragma once

#include<iostream>
#include<fstream>
#include<cmath>
#include<string>
#include<Eigen/Dense>
#include <nlohmann/json.hpp>
#include <algorithm>
#include "Vec2.hpp"
#include "utilities.hpp"
#include "ParamBank.hpp"
#include <ros/ros.h>

namespace v2_control{

  using json = nlohmann::json;


class Pure_Pursuit{


  public:
 
  Pure_Pursuit();
  Pure_Pursuit( const ParamBank &P);
  void setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);
  Track_data ptrack_;
  
  inline double get_y_error_in_vehicle_frame() const {return y_error_in_vehicle_frame;}
  geo_control_return PurePursuitControl(const State &bolid_state) ;
  private:
  
   Vec2 lookAheadPointInGlobalFrame;
   ParamBank param_;
   double y_error_in_vehicle_frame;
   void findLookAheadPoint(const State &bolid_state);
   double computeSteeringAngle(const State &bolid_state)  ;

};
}

#include "cone_detector.hpp"


namespace lem_dynamics_sim_{


       Track track_in_camera_frame( const State& state, const Track& track_global , const ParamBank& P){
           
           Track track_vehicle_frame;
           
           
           double cos_theta = std::cos(state.yaw);
           double sin_theta = std::sin(state.yaw);
           
           for(const auto& cone: track_global.cones)
          {
               Track_cone cone_vf;
               double dx = cone.x - state.x;
               double dy = cone.y - state.y;
               cone_vf.x = dx*cos_theta + dy*sin_theta + P.get("x_cog_to_camera"); ;
               cone_vf.y = -dx*sin_theta + dy*cos_theta + P.get("y_cog_to_camera");
               cone_vf.z = cone_vf.z + P.get("z_cog_to_camera");
               cone_vf.color = cone.color;
               cone_vf.distance = std::sqrt(cone_vf.x *cone_vf.x  + cone_vf.y * cone_vf.y +  cone_vf.z* cone_vf.z );
               track_vehicle_frame.cones.push_back(cone_vf);
           }
           
           return track_vehicle_frame;

       }

        Track shoot_a_frame(const Track&global_track, const ParamBank& P, const State& state){

        
            
          Track local_track = track_in_camera_frame( state, global_track , P);

          Track visible_track;

          double fov_y = P.get("fov_W"); // W - szerokość zdjęcia kątowa
          double fov_z = P.get("fov_H") ; // H - wysokość kątowa zdjęcia

          for(const auto& cone: visible_track.cones){
                // remove cones outisede of range and outside of FOV
                if(cone.distance > P.get("max_vision_range") || std::abs(cone.y) > std::tan(fov_y/2) * cone.distance ||  std::abs(cone.z) > std::tan(fov_z/2) * cone.distance ) {
                    visible_track.cones.erase(std::remove(visible_track.cones.begin(), visible_track.cones.end(), cone), visible_track.cones.end());
                }
          }
         //// === Prawdopodobieństwo wykrycia (model uproszczony) ===
          std::vector<Track_cone> detected_cones;
          std::random_device rd;
          std::mt19937 gen(rd());
          std::uniform_real_distribution<double> uniform01(0.0, 1.0);
          std::student_t_distribution<double> t_dist(6.0);  // t-Student df = 6
          for (const auto& cone : visible_track.cones) {
            double probability = std::exp(-cone.distance / (1.5 * P.get("camera_range")));
            if (uniform01(gen) < probability) {
                detected_cones.push_back(cone);
            }
          } 

        //// === Model szumu (t-Student df=6) dla błędów pomiaru pozycji ===
    double a = 14.4;   // współczynnik z artykułu (dla 2K)
    double b = 0.144;  // współczynnik z artykułu (dla 2K)

    for (auto& cone : detected_cones) {
        double RMSE = a * std::exp(b * cone.distance);
        double var_x = (RMSE * RMSE) / 2.0;
        double var_y = (RMSE * RMSE) / 2.0;

        // korekta wariancji t-Studenta (df=6 → var=6/(6-2)=1.5)
        double std_corr = std::sqrt(6.0 / (6.0 - 2.0));

        // losowy szum (niezależny w x i y)
        double noise_x = std::sqrt(var_x) * (t_dist(gen) / std_corr);
        double noise_y = std::sqrt(var_y) * (t_dist(gen) / std_corr);

        cone.x += noise_x;
        cone.y += noise_y;

        // aktualizacja odległości po dodaniu szumu
        cone.distance = std::sqrt(cone.x * cone.x + cone.y * cone.y + cone.z * cone.z);
    }

    visible_track.cones = detected_cones;
    return visible_track;
}

} // lem_dynamics_sim_
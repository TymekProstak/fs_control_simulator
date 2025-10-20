#pragma once

#include "uttilities.hpp"

namespace lem_dynamics_sim_{

    struct Track_cone {
        double x; // x position in global frame , or when used as a result of function track_in_camera_frame,  wr to camera
        double y; //y  position in global frame , or when used as a result of function track_in_camera_frame,  wr to camera
        double z; // z  position in global frame always 0  or when used as a result of function track_in_camera_frame,  wr to camera
        double distance; // non zero only if when track is considerd with respect to camera
        std::string color; // eg YELLOW,BLUE
        
    };

    struct Track {
        std::vector<Track_cone> cones;
        
    
    };
    
    

    Track load_track_from_csv(const std::string& filename); 
    std::vector<std::vector<std::string>> read_csv(const std::string& filename);

    
} // namespace lem_dynamics_sim_
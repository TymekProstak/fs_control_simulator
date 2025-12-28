#include "cone_track.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <stdexcept>

namespace lem_dynamics_sim_
{
    
    Track load_track_from_csv(const std::string& filename)
    {
        Track track;
        std::vector<std::vector<std::string>> data = read_csv(filename);
        
        for (const auto& row : data) {
            if (row.size() < 3) continue; // Ensure there are enough columns
            
            Track_cone cone;
            cone.x = std::stod(row[0]);
            cone.y = std::stod(row[1]);
            cone.z = 0.0; // Assuming z is always 0
            cone.color = row[2];
            
            track.cones.push_back(cone);
        }
        
        return track;
    }

    std::vector<std::vector<std::string>> read_csv(const std::string& filename) {
        std::vector<std::vector<std::string>> data;
        std::ifstream file(filename);

        if (!file.is_open()) {
            throw std::runtime_error("Could not open file: " + filename);
        }

        std::string line;
        while (std::getline(file, line)) {
            std::vector<std::string> row;
            std::stringstream ss(line);
            std::string cell;

            while (std::getline(ss, cell, ',')) {
                row.push_back(cell);
            }

            data.push_back(row);
        }

        file.close();
        return data;
    }

} // namespace lem_dynamics_sim_

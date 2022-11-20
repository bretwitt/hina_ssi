#ifndef HINA_SSI_PLUGIN_DEM_LOADER_H
#define HINA_SSI_PLUGIN_DEM_LOADER_H

#include "dem.h"

using namespace gazebo;

namespace hina {
    class DEMLoader {

    public:
        static std::shared_ptr<DEM> load_dem(const std::string &file) {

            std::vector<std::vector<double>> height_map;

            std::ifstream f;
            f.open(file);

            if (!f.is_open()) {
                std::cerr << "Error: File Open Failed" << std::endl;
            }

            std::stringstream ss;
            std::string line;
            uint32_t n;
            uint32_t m;
            double scale;

            std::getline(f, line, ',');
            n = std::stoi(line);
            std::getline(f, line, ',');
            m = std::stoi(line);
            std::getline(f, line, ',');
            scale = std::stod(line);

            std::shared_ptr<DEM> dem = std::make_shared<DEM>(n, m, scale);
            int i = 0;

            while (std::getline(f, line, ',')) {
                double z = std::stod(line);
                dem->load_vertex(i, z);
                i++;
            }

            if (i != n * m) {
                std::cerr << "Error loading DEM, incorrect number of vertices" << std::endl;
            }

            f.close();
            return dem;
        }
    };
}
#endif //HINA_SSI_PLUGIN_DEM_LOADER_H

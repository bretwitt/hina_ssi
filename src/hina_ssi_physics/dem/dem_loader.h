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
            double w_x;
            double w_y;

            double res;
            double target_res;

            std::getline(f, line, ',');
            w_x = std::stod(line);              // true width
            std::getline(f, line, ',');
            w_y = std::stod(line);              // true width
            std::getline(f, line, ',');
            res = std::stod(line);
            std::getline(f, line, ',');
            target_res = std::stod(line);

            std::shared_ptr<DEM> dem = std::make_shared<DEM>(FieldTrueDimensions { w_x, w_y },res);

            int i = 0;

            while (std::getline(f, line, ',')) {
                double z = std::stod(line);
                dem->load_vertex(i, z);
                i++;
            }

            f.close();

            dem->upsample(target_res);

            std::cout << dem->field->get_vertex_at_index(2,3)->v3 << std::endl;

            return dem;
        }
    };
}
#endif //HINA_SSI_PLUGIN_DEM_LOADER_H

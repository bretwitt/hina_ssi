#ifndef HINA_SSI_PLUGIN_SOIL_DATA_H
#define HINA_SSI_PLUGIN_SOIL_DATA_H

#include <gazebo/common/common.hh>
#include "soil_config.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace gazebo {

    struct VertexAttributes {

        double k_phi = 814000.0f; //814000.0f;
        double k_e = 7.8e7;
        double k_c = 20680.0f;
        double c = 3500;
        double phi = 0.55;

        bool isAir = true;
        Vector3d v3;
        Vector3d v3_0;

        /* Frame physics states */
        double plastic_flow;
        double sigma_yield;
        double s_p;
        double s_e;
        double sigma;
        Vector3d normal_dA;

        explicit VertexAttributes(const Vector3d& v3) {
            this->v3_0 = v3;
            this->v3 = v3;
        }

        ~VertexAttributes() = default;

    };

    struct BedRockVertexAttributes {

        double k_phi = 814000.0f; //814000.0f;
        double k_e = 7.8e7;
        double k_c = 20680.0f;
        double c = 3500;
        double phi = 0.55;

        bool isAir = true;

        Vector3d v3;
        Vector3d v3_0;

        /* Frame physics states */
        double plastic_flow;
        double sigma_yield;
        double s_p;
        double s_e;
        double sigma;
        Vector3d normal_dA;

        explicit BedRockVertexAttributes(const Vector3d& v3) {
            this->v3_0 = v3;
            this->v3 = v3;
        }

        ~BedRockVertexAttributes() = default;

    }; 

    struct TerrainDEM {

        //size of terrain
        int n;
        int m;
        std::vector<std::vector<VertexAttributes>> SoilVertices;
        std::vector<std::vector<BedRockVertexAttributes>> BedRockVertices;


        TerrainDEM(int width, int length, const Vector3d& v3, std::vector<std::vector<int>> labels) {
            n = width;
            m = length; 

        /* SoilVertices = std::vector <std::vector<SoilVertexAttributes>>(n);
            BedRockVertices = std::vector <std::vector<BedRockVertexAttributes>>(n);*/

            for (int i = 0; i < n; ++i)
            {
                /*SoilVertices[i] = std::vector<SoilVertexAttributes>(m);
                BedRockVertices[i] = std::vector<BedRockVertexAttributes>(m);*/
                std::vector<VertexAttributes> rowSoil;
                std::vector<BedRockVertexAttributes> rowBRock;

                for (int j = 0; j < m; ++j)
                {
                    struct VertexAttributes s(v3);

                    struct BedRockVertexAttributes b(v3);

                    if (labels[i][j] = 0) {
                        w.z = 0;
                        s.isAir = false;
                    }
                    else if (labels[i][j] = 1) {
                        v.z = 0;
                        b.isAir = false;
                    }
                    else {
                        w.z = 0;
                        s.isAir = false;
                    }
                    rowSoil.push_back(s);
                    rowBRock.push_back(w);

                }
                SoilVertices.push_back(rowSoil);
                BedRockVertices.push_back(rowBRock);

            }
        }
    };



    struct SoilData {

        /* Terrain parameters */
        uint32_t x_width = 5;
        uint32_t y_width = 5;
        double scale = 2.0;
        double angle = 0;

        /* Runtime */
        double x_offset = 0;
        double y_offset = 0;
        uint32_t *indices{};
        std::unordered_map<uint32_t, std::unordered_map<uint32_t, VertexAttributes*>>* soil_hashmap{};

        // Dynamic Footprint Parameter
        double B = 0;


        explicit SoilData(SoilConfig config) {
            this->x_width = config.x_width;
            this->y_width = config.y_width;
            this->scale = config.scale;
            this->angle = config.angle;
        }

        ~SoilData() {
            delete[] indices;

            for(uint32_t x = 0; x < x_width; x++) {
                for(uint32_t y = 0; y < y_width; y++) {
                    delete (*soil_hashmap)[x][y];
                }
            }
            delete[] soil_hashmap;
        };


        void init_field() {
            soil_hashmap = new std::unordered_map<uint32_t, std::unordered_map<uint32_t, VertexAttributes*>>();
            indices = new uint32_t[(x_width - 1) * (y_width - 1) * 3 * 2];
        }

        VertexAttributes* vertex_at_flattened_index(uint32_t idx) const {
            uint32_t x;
            uint32_t y;
            unflatten_index(idx, x, y);
            return (*soil_hashmap)[x][y];
        }

        void set_vertex_at_flattened_index(uint32_t idx, const VertexAttributes& vtx) const {
            uint32_t x;
            uint32_t y;
            unflatten_index(idx, x, y);
            auto _vtx = get_vertex_at_index(x, y);
            *_vtx = vtx;
        }

        void unflatten_index(uint32_t idx, uint32_t& x, uint32_t& y) const {
            x = idx % (x_width);
            y = floor(idx / x_width);
        }

        VertexAttributes* get_vertex_at_index(uint32_t x, uint32_t y) const {
            return (*soil_hashmap)[std::min(x, x_width - 1)][std::min(y, y_width - 1)];
        }

        void set_vertex_at_index(uint32_t x, uint32_t y, VertexAttributes* vtx) const {
            (*soil_hashmap)[x][y] = vtx;
        }

        void get_nearest_index(Vector2d vtx, uint32_t& x, uint32_t& y) const {
            x = (int)((vtx.X() / scale) - x_offset);
            y = (int)((vtx.Y() / scale) - y_offset);
        }

    };
}
#endif //HINA_SSI_PLUGIN_SOIL_DATA_H

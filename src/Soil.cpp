#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
using ignition::math::Vector3d;

namespace gazebo {
    class Soil {
    public:
        int x_width = 50;
        int y_width = 50;
        double x_offset = 0;
        double y_offset = 0;
        double scale = 1.0;
        Vector3d ** soil_field{};

        Soil() {
            this->soil_field = new Vector3d*[x_width];
            for(int i = 0; i < x_width; i++) {
                this->soil_field[i] = new Vector3d[y_width];
            }

            this->reset_soil();
        }

        ~Soil() {
            for(int i = 0; i < x_width; i++) {
                delete this->soil_field[i];
            }
            delete soil_field;
        }

        void reset_soil() {
            x_offset = -(double)x_width / 2;
            y_offset = -(double)y_width / 2;

            for(int i = 0; i < x_width; i++) {
                for(int j = 0; j < y_width; j++) {
                    soil_field[i][j] = Vector3d(i*scale + x_offset,j*scale + y_offset,0.0);
                }
            }
        }

        bool intersects(Vector3d vertex, Vector3d intersect_plane, Vector3d normal) {
            // AABB
        }
    };
}

#endif
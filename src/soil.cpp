#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
using ignition::math::Vector3d;

namespace gazebo {

    struct SoilData {
        int x_width = 50;
        int y_width = 50;
        double x_offset = 0;
        double y_offset = 0;
        double scale = 1.0;
    };

    class Soil {
    private:
        SoilData _data;

    public:
        Vector3d ** soil_field{};

        explicit Soil(SoilData _data) : Soil() {
            this->_data = _data;
        }

        Soil() {
            this->soil_field = new Vector3d*[this->_data.x_width];
            for(int i = 0; i < this->_data.x_width; i++) {
                this->soil_field[i] = new Vector3d[this->_data.y_width];
            }
            this->reset_soil();
        }

        ~Soil() {
            for(int i = 0; i < this->_data.x_width; i++) {
                delete this->soil_field[i];
            }
            delete soil_field;
        }

        SoilData getData() {
            return _data;
        }
        
        void reset_soil() {
            this->_data.x_offset = -(double)this->_data.x_width / 2;
            this->_data.y_offset = -(double)this->_data.y_width / 2;

            for(int i = 0; i < this->_data.x_width; i++) {
                for(int j = 0; j < this->_data.y_width; j++) {
                    soil_field[i][j] = Vector3d(i*this->_data.scale + this->_data.x_offset,j*this->_data.scale + this->_data.y_offset,0.0);
                }
            }
        }

        // bool intersects(ContactMesh* mesh) {}

        bool intersects(Vector3d vertex, Vector3d intersect_plane, Vector3d normal) {
            // AABB
        }
    };
}

#endif
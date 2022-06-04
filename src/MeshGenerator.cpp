#include <gazebo/common/common.hh>

namespace gazebo {
    class MeshGenerator {
    public:
        common::Mesh* generate_mesh() {
            auto mesh = new common::Mesh();
            return mesh;
        }
    };
}

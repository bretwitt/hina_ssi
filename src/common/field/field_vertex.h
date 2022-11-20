#ifndef HINA_SSI_PLUGIN_FIELD_VERTEX_H
#define HINA_SSI_PLUGIN_FIELD_VERTEX_H
#include <cstdint>
#include <memory>
#include <cmath>
#include <unordered_map>
#include <gazebo/common/common.hh>

using ignition::math::Vector3d;

namespace hina {
    template<class T>
    class FieldVertex {

    public:
        Vector3d v3;
        Vector3d v3_0;
        std::shared_ptr<T> v;

        explicit FieldVertex(const Vector3d &v3, const Vector3d &v3_0) {
            this->v3 = v3;
            this->v3_0 = v3_0;
            v = std::make_shared<T>();
        }

        explicit FieldVertex(const Vector3d &v3_0) : FieldVertex(v3_0, v3_0) {}
    };
}
#endif //HINA_SSI_PLUGIN_FIELD_VERTEX_H

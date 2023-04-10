#ifndef HINA_SSI_PLUGIN_TERRAMECHANICS_H
#define HINA_SSI_PLUGIN_TERRAMECHANICS_H

#include "../../common/field/field.h"
#include "../../common/geometry.h"
#include "../soil/soil_vertex.h"
#include "../soil/soil_physics_params.h"

namespace hina {
    class Terramechanics {
    public:
        static void terramx_deform(const Triangle &meshTri, uint32_t x, uint32_t y,
                            const std::shared_ptr<FieldVertex<SoilVertex>> &vertex, double w,
                            double &displaced_volume, SoilPhysicsParams vert_attr,
                            const std::shared_ptr<UniformField<SoilVertex>>& p_field,
                            Vector3d& force_v, Vector3d& force_origin
                            );

        static bool penetrates(const Triangle& meshTri, const std::shared_ptr<FieldVertex<SoilVertex>>& vtx, double w) {
            auto point = vtx->v3;
            return (meshTri.centroid().Z() <= point.Z() && intersects_projected(meshTri, AABB(point, w )));
        };


        static bool intersects_projected(const Triangle& meshTri, const AABB& vertexRect) {
            return Geometry::getInstance()->intersects_box_tri(meshTri, vertexRect) ;
        };

    };

}


#endif //HINA_SSI_PLUGIN_TERRAMECHANICS_H

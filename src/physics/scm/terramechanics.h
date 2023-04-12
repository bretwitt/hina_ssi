#ifndef HINA_SSI_PLUGIN_TERRAMECHANICS_H
#define HINA_SSI_PLUGIN_TERRAMECHANICS_H

#include "../../common/field/field.h"
#include "../../common/geometry.h"
#include "../soil/soil_vertex.h"
#include "../soil/soil_physics_params.h"

namespace hina {
    typedef struct {
        std::shared_ptr<FieldVertex<SoilVertex>> vtx_ul;
        std::shared_ptr<FieldVertex<SoilVertex>> vtx_dl;
        std::shared_ptr<FieldVertex<SoilVertex>> vtx_ur;
        std::shared_ptr<FieldVertex<SoilVertex>> vtx_dr;

        std::shared_ptr<FieldVertex<SoilVertex>> vtx_u;
        std::shared_ptr<FieldVertex<SoilVertex>> vtx_d;
        std::shared_ptr<FieldVertex<SoilVertex>> vtx_l;
        std::shared_ptr<FieldVertex<SoilVertex>> vtx_r;
    } VtxNeighbors;

    class Terramechanics {
    public:
        static void terramx_deform(const Triangle &meshTri, const std::shared_ptr<FieldVertex<SoilVertex>> &vertex,
                                   double w,double &displaced_volume, SoilPhysicsParams vert_attr, VtxNeighbors neighbors,
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

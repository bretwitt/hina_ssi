#ifndef HINA_SSI_PLUGIN_VISUAL_CHUNK_H
#define HINA_SSI_PLUGIN_VISUAL_CHUNK_H

#include "../common/field/uniform_field.h"
#include <gazebo/common/common.hh>
#include "ogre_soil_renderer.cpp"
#include <gazebo/rendering/rendering.hh>

using gazebo::msgs::Vector3d;

namespace hina {
    class VisualChunk {

    static int id;

    private:
        std::shared_ptr<OgreSoilRenderer> p_ogre_soil_renderer = nullptr;
        std::shared_ptr<UniformField<ColorAttributes>> field;
        bool init_viz = false;
        bool soil_initialized = false;

    public:
        std::vector<gazebo::msgs::Vector3d> field_v;
        rendering::VisualPtr visual = nullptr;

        void init_visual_chunk(rendering::VisualPtr visualPtr, uint32_t verts_x, uint32_t verts_y) {
            field = std::make_shared<UniformField<ColorAttributes>>(FieldVertexDimensions{verts_x,verts_y}, 1);
            field->init_field();
            visual = visualPtr;
        }

        void update() {
            if(!init_viz) {
                init_soil(field);
                init_viz = true;
                return;
            }
            if(soil_initialized) {
                update_soil_mesh(field);
            }
        }

        void init_soil(const std::shared_ptr<UniformField<ColorAttributes>>& p_field) {
            p_ogre_soil_renderer = std::make_shared<OgreSoilRenderer>();
            p_ogre_soil_renderer->setScenePtr(visual->GetScene());
            p_ogre_soil_renderer->create_ogre_mesh(p_field, id++);
        }

        void update_soil_mesh(const std::shared_ptr<UniformField<ColorAttributes>>& p_soil) {
            if(p_ogre_soil_renderer != nullptr) {
                p_ogre_soil_renderer->update_ogre_mesh(p_soil);
            }
        }
    };
}


#endif //HINA_SSI_PLUGIN_VISUAL_CHUNK_H

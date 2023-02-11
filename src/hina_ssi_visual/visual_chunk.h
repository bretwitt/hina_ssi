#ifndef HINA_SSI_PLUGIN_VISUAL_CHUNK_H
#define HINA_SSI_PLUGIN_VISUAL_CHUNK_H

#include "../common/field/uniform_field.h"
#include <gazebo/common/common.hh>
#include "ogre_soil_renderer.cpp"
#include <gazebo/rendering/rendering.hh>

using gazebo::msgs::Vector3d;

namespace hina {
    class VisualChunk {

    private:
        std::shared_ptr<OgreSoilRenderer> p_ogre_soil_renderer = nullptr;
        std::shared_ptr<UniformField<ColorAttributes>> field = nullptr;
        bool init_viz = false;
        bool soil_initialized = false;
        int i = 0;
        int j = 0;

    public:
        std::vector<gazebo::msgs::Vector3d> field_v;
        uint32_t verts_x,verts_y;
        rendering::VisualPtr visual = nullptr;

        VisualChunk() {
            field_v = std::vector<gazebo::msgs::Vector3d>();
        }

        ~VisualChunk() {
            p_ogre_soil_renderer->delete_ogre_mesh();
        }

        void init_visual_chunk(rendering::VisualPtr visualPtr, uint32_t verts_x, uint32_t verts_y, int i, int j) {
            field = std::make_shared<UniformField<ColorAttributes>>(FieldVertexDimensions{verts_x,verts_y}, 1);
            field->init_field();
            visual = visualPtr;
            this->i = i;
            this->j = j;
        }

        void update() {
            if(!init_viz && field != nullptr) {
                init_soil(field);
                init_viz = true;
                return;
            }
            else if( init_viz ){
                update_soil_mesh(field);
            }
        }

        void update_field(const gazebo::msgs::Vector3d& _v, uint32_t i) {
            field->set_vertex_at_flattened_index(i, FieldVertex<ColorAttributes>(Vector3d(_v.x(), _v.y(), _v.z())));
        }

        void init_soil(std::shared_ptr<UniformField<ColorAttributes>> p_field) {
            p_ogre_soil_renderer = std::make_shared<OgreSoilRenderer>();
            p_ogre_soil_renderer->setScenePtr(visual->GetScene());
            std::string id = std::to_string(i) + " " + std::to_string(j);
            p_ogre_soil_renderer->create_ogre_mesh(p_field, id);
        }

        void update_soil_mesh(const std::shared_ptr<UniformField<ColorAttributes>>& p_soil) {
            if(p_ogre_soil_renderer != nullptr) {
                p_ogre_soil_renderer->update_ogre_mesh(p_soil);
            }
        }
    };
}


#endif //HINA_SSI_PLUGIN_VISUAL_CHUNK_H

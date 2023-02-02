#ifndef HINA_SSI_PLUGIN_CPP
#define HINA_SSI_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <utility>
#include "ogre_soil_renderer.cpp"
#include "Soil.pb.h"
#include "../hina_ssi_physics/soil/soil_chunk_location_metadata.h"

using namespace gazebo;

namespace hina {

    class HinaSSIVisualPlugin : public VisualPlugin {

    private:
        std::shared_ptr<OgreSoilRenderer> p_ogre_soil_renderer = nullptr;

        event::ConnectionPtr connectionPtr = nullptr;
        rendering::VisualPtr visual = nullptr;
        rendering::VisualPtr terrainViz = nullptr;
        transport::NodePtr node = nullptr;
        transport::SubscriberPtr sub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;

        std::shared_ptr<UniformField<ColorAttributes>> field;

        bool soil_initialized = false;
        bool init_viz = false;
        std::unordered_map<int,std::unordered_map<int,
            std::shared_ptr<UniformField<ColorAttributes>>>> map;

        std::vector<std::shared_ptr<UniformField<ColorAttributes>>> active_chunks;


    public:
        HinaSSIVisualPlugin() : VisualPlugin() {
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {
            connectionPtr = event::Events::ConnectRender(boost::bind(&HinaSSIVisualPlugin::update, this));
            init_transport();
            visual = _visual;
        }

        void init_transport() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            sub = node->Subscribe("~/soil", &HinaSSIVisualPlugin::OnSoilUpdate, this);
        }

        void OnSoilUpdate(const boost::shared_ptr<const hina_ssi_msgs::msgs::Soil> &soil_update) {

            int vert_x = soil_update->len_col();
            int vert_y = soil_update->len_row();

            auto v = soil_update->chunk_field();
            auto id = soil_update->id_field();

            for(const gazebo::msgs::Vector2d& id0 : id) {
                auto id_vec = Vector2d( id0.x(), id0.y() );
                if(map[id0.x()][id0.y()] == nullptr) {
                    map[id0.x()][id0.y()] = std::make_shared<UniformField<ColorAttributes>>(FieldVertexDimensions { static_cast<double>(vert_x), static_cast<double>(vert_y) }, 1);
                    map[id0.x()][id0.y()]->init_field();
                    active_chunks.push_back(map[id0.x()][id0.y()]);
                }
            }
//
//            uint32_t i = 0;
//            for(const gazebo::msgs::Vector3d& v0 : v) {
//                auto vert = FieldVertex<ColorAttributes>(Vector3d(v0.x(), v0.y(), v0.z()));
//                auto vtx_idx = i / (vert_x*vert_y);
//                ->set_vertex_at_flattened_index(vtx_idx, vert);
//                i++;
//            }
//
//            if(!soil_initialized) {
//                init_viz = true;
//                soil_initialized = true;
//            }
        }

        void update() {
            if(init_viz) {
                init_soil(field);
                init_viz = false;
                return;
            }
            if(soil_initialized) {
                update_soil_mesh(field);
            }
        }

        void init_soil(const std::shared_ptr<UniformField<ColorAttributes>>& p_field) {
            p_ogre_soil_renderer = std::make_shared<OgreSoilRenderer>();
            p_ogre_soil_renderer->setScenePtr(visual->GetScene());
            p_ogre_soil_renderer->create_ogre_mesh(p_field);
        }

        void update_soil_mesh(const std::shared_ptr<UniformField<ColorAttributes>>& p_soil) {
            p_ogre_soil_renderer->update_ogre_mesh(p_soil);
        }
    };

    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIVisualPlugin)

}

#endif
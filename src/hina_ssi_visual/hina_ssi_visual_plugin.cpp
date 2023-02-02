#ifndef HINA_SSI_PLUGIN_CPP
#define HINA_SSI_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <utility>
#include "ogre_soil_renderer.cpp"
#include "Soil.pb.h"
#include "../hina_ssi_physics/soil/soil_chunk_location_metadata.h"
#include "../common/field/chunked_field.h"
#include "visual_chunk.h"

using namespace gazebo;

namespace hina {

    class HinaSSIVisualPlugin : public VisualPlugin {

    private:

        event::ConnectionPtr connectionPtr = nullptr;
        rendering::VisualPtr visual = nullptr;
        rendering::VisualPtr terrainViz = nullptr;
        transport::NodePtr node = nullptr;
        transport::SubscriberPtr sub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;

        ChunkedField<std::shared_ptr<VisualChunk>> chunks;

        bool soil_initialized = false;
        bool init_viz = false;
        std::unordered_map<int,std::unordered_map<int,
            std::shared_ptr<UniformField<ColorAttributes>>>> map;

        uint32_t verts_x;
        uint32_t verts_y;

    public:
        HinaSSIVisualPlugin() : VisualPlugin() {
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {
            connectionPtr = event::Events::ConnectRender(boost::bind(&HinaSSIVisualPlugin::update, this));
            init_transport();
            init_chunk_field();
            visual = _visual;

        }

        void init_transport() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            sub = node->Subscribe("~/soil", &HinaSSIVisualPlugin::OnSoilUpdate, this);
        }

        void init_chunk_field() {
            chunks.register_chunk_create_callback(boost::bind(&HinaSSIVisualPlugin::OnChunkCreated, this, _1, _2));
        }

        void OnSoilUpdate(const boost::shared_ptr<const hina_ssi_msgs::msgs::Soil> &soil_update) {
            int vert_x = soil_update->len_col();
            int vert_y = soil_update->len_row();

            this->verts_x = vert_x;
            this->verts_y = vert_y;

            auto v = soil_update->chunk_field();
            auto id = soil_update->id_field();

            chunks.pre_update();

            for(const gazebo::msgs::Vector2d& id0 : id) {
                chunks.poll_chunk({ id0.x(), id0.y() });
            }

            uint32_t i = 0;
            uint32_t size = vert_x*vert_y;
            for(const gazebo::msgs::Vector2d& id0 : id) {
                auto start = i*size;
                auto end = start + size;
                auto c = chunks.get_chunk_cont({id0.x(), id0.y()});
                if( c != nullptr ) {
                    std::copy(v.begin() + start, v.begin() + end, std::back_inserter(c->field_v));
                }
                i++;
            }

            if(!soil_initialized) {
                init_viz = true;
                soil_initialized = true;
            }

            chunks.post_update();
        }

        std::shared_ptr<VisualChunk> OnChunkCreated(int i, int j) {
            auto vc = std::make_shared<VisualChunk>();
            vc->init_visual_chunk(visual, this->verts_x, this->verts_y);
            return vc;
        }

        void update() {
            if(soil_initialized) {
                for(auto chunk : chunks.get_active_chunks()) {
                    chunk->container->update();
                }
            }
        }
    };

    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIVisualPlugin)

}

#endif
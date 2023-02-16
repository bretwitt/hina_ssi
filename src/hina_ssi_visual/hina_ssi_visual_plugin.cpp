#ifndef HINA_SSI_PLUGIN_CPP
#define HINA_SSI_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <utility>
#include "ogre_soil_renderer.cpp"
#include "SoilChunk.pb.h"
#include "../hina_ssi_physics/soil/soil_chunk_location_metadata.h"
#include "../common/field/chunked_field.h"
#include "visual_chunk.h"

using namespace gazebo;
using ignition::math::Vector2d;

namespace hina {
    class HinaSSIVisualPlugin : public VisualPlugin {

    private:

        event::ConnectionPtr connectionPtr = nullptr;
        rendering::VisualPtr visual = nullptr;
        rendering::VisualPtr terrainViz = nullptr;
        transport::NodePtr node = nullptr;
        transport::SubscriberPtr sub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;

        common::Time time;
        double sec{};
        double last_sec{};
        double last_sec_viz{};

        ChunkedField<std::shared_ptr<VisualChunk>> chunks;

        bool soil_initialized = false;

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

        void OnSoilUpdate(const boost::shared_ptr<const hina_ssi_msgs::msgs::SoilChunk> &soil_update) {
            uint32_t x_verts = soil_update->len_col();
            uint32_t y_verts = soil_update->len_row();

            uint32_t i = soil_update->id_i();
            uint32_t j = soil_update->id_j();

            this->verts_x = x_verts;
            this->verts_y = y_verts;

            auto v = soil_update->chunk_field();

            chunks.poll_chunk( ChunkedFieldLocation { i,j });

            auto c = chunks.get_chunk(ChunkedFieldLocation{i,j});

            uint32_t count = 0;
            for(auto& vert : soil_update->chunk_field()) {
                c->container->update_field(vert,count++);
            }
        }

        std::shared_ptr<VisualChunk> OnChunkCreated(int i, int j) {
            auto vc = std::make_shared<VisualChunk>();
            vc->init_visual_chunk(visual, this->verts_x, this->verts_y, i, j);
            return vc;
        }

        void update() {
            time = common::Time::GetWallTime();

            sec = time.Double();

            double dt = sec - last_sec;

            if (dt > 10) {
                chunks.post_update();
                chunks.pre_update();
                last_sec = sec;
            }
            for(auto chunk : chunks.get_active_chunks()) {
                chunk->container->update();
            }
        }
    };
    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIVisualPlugin)
}
#endif
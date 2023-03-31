#ifndef HINA_SSI_PLUGIN_CPP
#define HINA_SSI_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <utility>
#include "ogre_soil_renderer.cpp"
#include "SoilChunk.pb.h"
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

        ChunkedField<std::shared_ptr<VisualChunk>> chunks;

        std::unordered_map<int,std::unordered_map<int,
            std::shared_ptr<UniformField<ColorAttributes>>>> map;

        std::vector<double> flow_v;

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

            int i = soil_update->id_i();
            int j = soil_update->id_j();

            this->verts_x = x_verts;
            this->verts_y = y_verts;

            auto v = soil_update->chunk_field();

            chunks.poll_chunk( ChunkedFieldLocation { i,j });

            auto c = chunks.get_chunk(ChunkedFieldLocation{i,j});

            uint32_t count = 0;
            for(auto& vert : soil_update->chunk_field()) {
                c->container->update_field(vert,count++);
            }
            count = 0;
            for(auto &vert : soil_update->footprint_field()) {
                float r;
                float g;
                float b;
                get_rgb_from_sp(vert,r,g,b);
                c->container->set_vtx_color(count++,r,g,b);
            }

            count = 0;
            for(auto& vert : soil_update->normals_field()) {
                c->container->update_field_normal(vert,count++);
            }
        }

        void get_rgb_from_sp(int footprint, float& r, float& g, float& b) {
//            std::cout << s_p << " " << frame_flow_max << std::endl;
            if(footprint == 1) {
                r = 1;
                g = 0;
                b = 0;
            } else if (footprint == 2) {
                r = 0;
                g = 1;
                b = 0;
            } else {
                r = 1;
                g = 1;
                b = 1;
            }
            
            //std::cout << b << std::endl;
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
            for(const auto& chunk : chunks.get_active_chunks()) {
                chunk->container->update();
            }
        }
    };
    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIVisualPlugin)
}
#endif
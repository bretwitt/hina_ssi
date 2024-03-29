#ifndef HINA_SSI_PLUGIN_CPP
#define HINA_SSI_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <utility>
#include "ogre_soil_renderer.cpp"
#include "SoilChunk.pb.h"
#include "BodyPhysics.pb.h"
#include "Triangles.pb.h"
#include "../common/field/chunked_field.h"
#include "visual_chunk.h"
#include "visual_triangles.h"
#include "visual_body_physics.h"
#include "triangle_info_renderer.cpp"
#include "body_info_renderer.cpp"

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
        transport::SubscriberPtr tri_sub = nullptr;
        transport::SubscriberPtr body_sub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;
        event::ConnectionPtr preRenderPtr = nullptr;

        common::Time time;
        double sec{};
        double last_sec{};

        ChunkedField<std::shared_ptr<VisualChunk>> chunks;

        VisualTriangles triangles{};
        TriangleInfoRenderer renderer;

        BodyInfoRenderer bp_renderer;
        VisualBodyPhysics body_physics;

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
            preRenderPtr = event::Events::ConnectPreRender(boost::bind(&HinaSSIVisualPlugin::prerender, this));
            init_transport();
            init_chunk_field();
            visual = _visual;
        }


        void init_transport() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            sub = node->Subscribe("~/soil", &HinaSSIVisualPlugin::OnSoilUpdate, this);
            tri_sub = node->Subscribe("~/triangles", &HinaSSIVisualPlugin::OnTrianglesUpdate, this);
            body_sub = node->Subscribe("~/body_physics", &HinaSSIVisualPlugin::OnBodyPhysicsUpdate, this);
        }

        void init_chunk_field() {
            chunks.register_chunk_create_callback(boost::bind(&HinaSSIVisualPlugin::OnChunkCreated, this, _1, _2));
        }


        void OnBodyPhysicsUpdate(const boost::shared_ptr<const hina_ssi_msgs::msgs::BodyPhysics> &bp_update) {
            auto len = bp_update->len();
            auto traction_v = bp_update->traction_force();
            auto normal_v = bp_update->normal_force();
            auto origin_v = bp_update->force_origin();
            VisualBodyPhysics physics{};

            for(auto& traction : traction_v) {
                physics.traction.push_back(traction);
            }
            for(auto& normal : normal_v) {
                physics.normal.push_back(normal);
            }
            for(auto& origin : origin_v) {
                physics.origin.push_back(origin);
            }

            body_physics = physics;
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

        void OnTrianglesUpdate(const boost::shared_ptr<const hina_ssi_msgs::msgs::Triangles> &tri_update) {
            uint32_t n = tri_update->len_triangles();

            VisualTriangles tris{};
            auto centroids = tri_update->centroids();
            auto forces = tri_update->forces();
            auto slip_velocity = tri_update->slip_velocity();
            auto normals = tri_update->normal();
            auto contacts = tri_update->contact();
            auto shear_displ = tri_update->shear_displacement();

            for(auto& center : centroids) {
                tris.centroid.push_back(center);
            }
            for(auto& force : forces) {
                tris.forces.push_back(force);
            }
            for(auto& vel : slip_velocity) {
                tris.slip_velocity.push_back(vel);
            }
            for(auto& normal : normals) {
                tris.normal.push_back(normal);
            }
            for(auto& contact : contacts){
                tris.contact.push_back(contact);
            }
            for(auto& displ : shear_displ) {
                tris.shear_displacement.push_back(displ);
            }

            triangles = tris;
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

        void prerender() {
            renderer.init(visual);
            renderer.update_triangles(triangles);

            bp_renderer.init(visual);
            bp_renderer.update_body(body_physics);
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
#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#define PHYS_PROFILER 0
#include "soil/footprint.h"

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/common/Profiler.hh>
#include <memory>
#include <utility>

#include "soil/soil.h"
#include "dem/dem_loader.h"
#include "../common/field/uniform_field.h"
#include "soil/triangle_context.h"
#include "soil/body_context.h"

#include "Triangles.pb.h"
#include "SoilChunk.pb.h"
#include "BodyPhysics.pb.h"

namespace hina {

class HinaSSIWorldPlugin : public WorldPlugin {

private:
    std::shared_ptr<Soil> p_soil = nullptr;
    std::unique_ptr<msgs::Vector3d[]> p_soil_v = nullptr;
    std::unique_ptr<msgs::Vector2d[]> p_soil_id_v = nullptr;

    transport::NodePtr node = nullptr;
    transport::PublisherPtr soilPub = nullptr;
    transport::PublisherPtr triPub = nullptr;
    transport::PublisherPtr bodyPub = nullptr;
    event::ConnectionPtr updateEventPtr = nullptr;
    event::ConnectionPtr onEntityAddedEventPtr = nullptr;
    physics::WorldPtr world = nullptr;
    sdf::ElementPtr sdf = nullptr;

    std::map<physics::LinkPtr, const common::Mesh *> mesh_lookup{};
    std::unordered_map<const common::Mesh*,double> footprint_lookup{};

    std::vector<std::pair<TriangleContext,Footprint>> triangle_states;

    transport::Node* trspt = nullptr;

    // TODO: Temporary... move after grind is over
    struct tuple_hash {
        template <class T1, class T2, class T3>
        std::size_t operator() (const std::tuple<T1, T2, T3>& tuple) const {
            const auto& [a, b, c] = tuple;
            std::size_t h1 = std::hash<T1>{}(a);
            std::size_t h2 = std::hash<T2>{}(b);
            std::size_t h3 = std::hash<T3>{}(c);

            return h1 ^ h2 ^ (h3 << 1);
        }
    };
    std::unordered_map<std::tuple<int, int, int>, std::pair<double,double>, tuple_hash> shear_displacement_map; // Hashmap to store shear displacement per triangle

    std::vector<BodyContext> body_contexts{};


    common::Time time;
    common::Time vizTime;

    double sec{};
    double vsec{};
    double last_sec{};
    double last_sec_viz{};
    double last_sec_tri{};
    int col_threads = 3;

public:

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
        updateEventPtr = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&HinaSSIWorldPlugin::update, this));
        onEntityAddedEventPtr = event::Events::ConnectAddEntity(
                boost::bind(&HinaSSIWorldPlugin::OnEntityAdded, this, _1));
        world = _world;
        sdf = _sdf;
        init_threading();
        init_soil();
        init_transport();
        init_models();

        std::cout << "Finished loading TaRO-SCM" << std::endl;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ~HinaSSIWorldPlugin() {
        delete trspt;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void init_threading() {
        if (sdf->HasElement("col_threads")) {
            col_threads = sdf->GetElement("col_threads")->Get<int>();
        }
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void init_soil() {
        if (sdf->HasElement("dem")) {
            init_dem();
        } else if(sdf->HasElement("sandbox")){
            init_sandbox();
        }
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void init_transport() {
        trspt = new transport::Node();
        this->node = transport::NodePtr(trspt);
        node->Init();
        soilPub = node->Advertise<hina_ssi_msgs::msgs::SoilChunk>("~/soil");
        triPub = node->Advertise<hina_ssi_msgs::msgs::Triangles>("~/triangles");
        bodyPub = node->Advertise<hina_ssi_msgs::msgs::BodyPhysics>("~/body_physics");
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void init_models() {
        auto model_v = world->Models();
        for (const auto &model: model_v) {
            init_links(model->GetName());
        }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void OnEntityAdded(const std::string &str) {
        init_links(str);
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void init_links(const std::string &model_name) {

        std::vector<std::string> links;
        auto link_element_ptr = sdf->GetElement("links");
        if (link_element_ptr != nullptr) {
            link_element_ptr = link_element_ptr->GetFirstElement();
            if (link_element_ptr != nullptr) {
                links.push_back(link_element_ptr->Get<std::string>());
            }
            while (link_element_ptr != nullptr) {
                link_element_ptr = link_element_ptr->GetNextElement("link");
                if (link_element_ptr != nullptr) {
                    links.push_back(link_element_ptr->Get<std::string>());
                }
            }
        }

        auto model = world->EntityByName(model_name)->GetParentModel();
        auto link_v = model->GetLinks();
        for (const auto &link: link_v) {

            if (std::end(links) == std::find(std::begin(links), std::end(links), link->GetName())) continue;

            auto collElemPtr = link->GetSDF()->GetElement("collision");
            if (collElemPtr != nullptr) {
                if (!collElemPtr->HasElement("geometry")) break;
                auto geomElemPtr = collElemPtr->GetElement("geometry");
                if (geomElemPtr != nullptr) {
                    auto meshElemPtr = geomElemPtr->GetElement("mesh");
                    if (!geomElemPtr->HasElement("mesh")) break;
                    if (meshElemPtr != nullptr) {
                        auto uriElemPtr = meshElemPtr->GetElement("uri");
                        if (!meshElemPtr->HasElement("uri")) break;
                        if (uriElemPtr != nullptr) {
                            auto model_name = model->GetName();
                            auto uri = uriElemPtr->Get<std::string>();
                            load_mesh(link, uri);

                            std::cout << "Loaded " << model_name << ": " << link->GetName() << " uri..." << uri << std::endl;
                        }
                    }
                }
            }
        }
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void init_sandbox() {
        auto params = sdf->GetElement("params");

        auto k_phi = params->GetElement("k_phi")->Get<double>();
        auto k_e = params->GetElement("k_e")->Get<double>();
        auto c = params->GetElement("c")->Get<double>();
        auto phi = params->GetElement("phi")->Get<double>();
        auto mfr = params->GetElement("mfr")->Get<double>();

        auto soil_params = SoilPhysicsParams { k_phi, k_e, 0, c, phi, mfr };

        auto sandbox_elem = sdf->GetElement("sandbox");
        auto x_width = sandbox_elem->GetElement("width_x")->Get<double>();
        auto y_width = sandbox_elem->GetElement("width_y")->Get<double>();
        auto scale = sandbox_elem->GetElement("resolution")->Get<double>();
        auto angle = sandbox_elem->GetElement("angle")->Get<double>();

        auto sandbox_sampler = std::make_shared<SandboxVertexSampler>(angle,soil_params);
        this->p_soil = std::make_shared<Soil>(sandbox_sampler, FieldTrueDimensions {x_width, y_width }, scale);
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void init_dem() {
        auto dem_elem = sdf->GetElement("dem");

        std::string filename = dem_elem->GetElement("file")->Get<std::string>();
        std::string file_name = gazebo::common::SystemPaths::Instance()->FindFile(filename);
        auto dem = DEMLoader::load_dem_from_geotiff(file_name);

        /*
        if(dem_elem->HasElement("upscale_res")) {
            auto upscale_res = dem_elem->GetElement("upscale_res")->Get<double>();
            dem->upsample(upscale_res);
        }
         */

        auto dem_sampler = std::make_shared<DEMVertexSampler>(dem);
        this->p_soil = std::make_shared<Soil>(dem_sampler, FieldTrueDimensions {0.5, 0.5 }, dem->field->scale);
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void load_mesh(const physics::LinkPtr &link, const std::string &mesh_uri) {
        auto mesh = common::MeshManager::Instance()->Load(mesh_uri);
        mesh_lookup.insert(std::pair<physics::LinkPtr, const common::Mesh *>(link, mesh));
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void update() {

        time = this->world->SimTime();
        vizTime = common::Time::GetWallTime();

        sec = time.Double();
        vsec = vizTime.Double();

#ifdef PHYS_PROFILER
        IGN_PROFILE_BEGIN("TAROSCM::Soil Physics Update");
#endif
        double dt = sec - last_sec;
        double dt_viz = vsec - last_sec_viz;
        double dt_tri = vsec - last_sec_tri;

        update_soil(p_soil, dt);

#ifdef PHYS_PROFILER
        IGN_PROFILE_END();
        IGN_PROFILE_BEGIN("TAROSCM::Broadcast Soil");
#endif
        last_sec = sec;

        if (dt_viz > (1. / 1.f)) {
            broadcast_soil(p_soil);
            last_sec_viz = vsec;
        }

        if(dt_tri > (1./ 5.f)) {
            broadcast_triangles(triangle_states);
            broadcast_body(body_contexts);
            last_sec_tri = vsec;
        }

#ifdef PHYS_PROFILER
        IGN_PROFILE_END();
#endif
    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void update_soil(const std::shared_ptr<Soil>& soil, double dt) {


        triangle_states.clear(); // For transport
        soil->pre_update();

        // Loop through each mesh in the table
        int i = 0;
        for (auto &iter: mesh_lookup) {
            auto link = iter.first;
            auto mesh = iter.second;
            auto B = footprint_lookup[mesh];

            if(B = 0) {
                B = 9999;
            }

            double max_sinkage = 1;

            Vector3d traction_force;
            Vector3d normal_force;
            Vector3d origin;

            // Iterate through each submesh
            int no_submeshes = 1; // mesh->GetSubMeshCount()
            for (uint32_t i = 0; i < no_submeshes; i++) {

                auto submesh = mesh->GetSubMesh(i);
                auto indices = submesh->GetIndexCount();

                auto col_pose = link->GetCollision(0.)->RelativePose();
                auto cpos = col_pose.Pos();
                auto crot = col_pose.Rot();

                auto pose = link->WorldPose();
                auto rot = pose.Rot();
                auto pos = pose.Pos();

                auto aabb = link->GetCollision(0.)->BoundingBox();
                auto max = aabb.Max();
                auto min = aabb.Min();

                soil->query_chunk((max+min) / 2);
                //soil->query_chunk(pos);

                std::vector<Footprint> mesh_footprint;

                double total_footprint_size = 0.0;

                auto j = shear_displacement_map[std::make_tuple(0,1,2)].second; // Shear displacment in previous frame
                auto d = shear_displacement_map[std::make_tuple(0,1,2)].first; // Slip in previous frame


                // Vertex level computations
#ifdef PHYS_PROFILER
                IGN_PROFILE_BEGIN("Collision Detection");
#endif
                #pragma omp parallel num_threads(col_threads) default(none) shared(std::cout, B,max_sinkage, traction_force, normal_force, shear_displacement_map, mesh_footprint /*, total_footprint_size*/) firstprivate(d,j,submesh, soil, crot, cpos, pos, rot, indices, dt, link)
                {
                    #pragma omp for nowait schedule(guided) //reduction(+:total_footprint_size)

                     // Iterate through triangles in the submesh
                     for (uint32_t idx_unrolled = 0; idx_unrolled < (indices / 3); idx_unrolled++) {

                        auto idx = idx_unrolled * 3;

                        auto v0 = submesh->Vertex(submesh->GetIndex(idx));
                        auto v1 = submesh->Vertex(submesh->GetIndex(idx + 1));
                        auto v2 = submesh->Vertex(submesh->GetIndex(idx + 2));

                        auto tuple = std::make_tuple(idx,idx + 1,idx + 2);

                        auto cv0 = crot.RotateVector(v0) + cpos;
                        auto cv1 = crot.RotateVector(v1) + cpos;
                        auto cv2 = crot.RotateVector(v2) + cpos;

                        auto c1v0 = rot.RotateVector(cv0) + pos;
                        auto c1v1 = rot.RotateVector(cv1) + pos;
                        auto c1v2 = rot.RotateVector(cv2) + pos;

                        auto meshTri = Triangle(c1v0, c1v1, c1v2);
                        auto meshTriBody = Triangle(cv0,cv1,cv2);

                        // Calculate slip velocity
                        auto center = meshTri.centroid();
                        auto angular_vel = link->RelativeAngularVel();
                        Vector3d e2q = rot.Euler();
                        auto no_x_rot = Vector3d(0,e2q.Y(),e2q.Z());
                        auto rot_pres_fwd = ignition::math::Quaternion(no_x_rot);
                        auto r_vec = center - link->WorldCoGPose().Pos();
                        auto slip_velocity = -angular_vel.Cross(r_vec);
                        auto slip_vel_f = rot_pres_fwd.RotateVector(slip_velocity);
                        auto V_j = slip_velocity.Length();

                        // Update shear displacement by one timestep
                        auto j_p_o = j + (V_j*dt);

                        //
                        auto tri_ctx = TriangleContext { meshTri, j_p_o,  slip_vel_f,
                                                     angular_vel, B };


                        // Compute contact forces and update soil graph
                        Footprint tri_ftp;
                        tri_ftp = soil->try_deform(tri_ctx, link);

                        // Reset shear displacement if not in contact
                        if(tri_ftp.getSize() == 0) {
                            j_p_o *= 0;
                        }

                        // Reset shear displacement if switching dir
                        auto dir = -angular_vel.X()/abs(angular_vel.X());
                        if(dir != d/abs(d)) {
                            j_p_o *= 0;
                        }

                        #pragma omp critical
                        {
                            triangle_states.push_back({tri_ctx,tri_ftp});
                            mesh_footprint.push_back(tri_ftp);
                            shear_displacement_map[tuple] = std::make_pair(dir,j_p_o);
                            j = shear_displacement_map[std::make_tuple(idx+3,idx+4,idx+5)].second;
                            d = shear_displacement_map[std::make_tuple(idx+3,idx+4,idx+5)].first;

                            traction_force += tri_ftp.force_x;
                            normal_force += tri_ftp.force_z;

                            if(max_sinkage > tri_ftp.max_sinkage) {
                                max_sinkage = tri_ftp.max_sinkage;
                            }
                        }
                     }
                }
#ifdef PHYS_PROFILER
                IGN_PROFILE_END();
#endif
                // Compute footprint stage for entire mesh
                soil->compute_footprint_stage(mesh_footprint,total_footprint_size,B);
                footprint_lookup[mesh] = B;
            }

//            Vector3d contact_force = Vector3d(0,0,normal_force.Z())+traction_force;

            // Get the orientation of the wheel link in the world frame
            ignition::math::Quaternion orientation = link->WorldPose().Rot();
            Vector3d ib = Vector3d(0,0,1);
            Vector3d jb = orientation.RotateVector(Vector3d(1,0,0));
            Vector3d kb = ib.Cross(jb).Normalize();
            Vector3d vel = link->WorldCoGLinearVel();

            double vi = vel.Dot(ib);
            double vj = vel.Dot(jb);
            double vk = vel.Dot(kb);

//            if(link->RelativeAngularVel().Length() < 0.01) {
//                traction_force *= kb/0.01;
//            }

            Vector3d contact_force = normal_force+traction_force;
            link->AddForce(contact_force);


            bool fwd = (vel.Dot(kb) < 0);
            bool rgt = (vel.Dot(jb) < 0);
            int dir = (fwd) ? 1 : -1;
            int ldir = (rgt) ? 1: -1;
            double R = (8.14e5*0.3+1.37e3)*(pow(max_sinkage,2)*0.5);
            double R_l = R;

            if(abs(vk) < 0.001) {
                R *= vk*100;
            }

            if(max_sinkage <= 0) {
                link->AddForce(R*kb*dir);
            }

            Vector3d dampingForce_jb = -500 * vj * jb; // Dampen +x
            Vector3d dampingForce_ib = -0 * vi * ib;  // Dampen +z
            link->AddForce(dampingForce_ib+dampingForce_jb);

            if(body_contexts.size() < i + 1 ) {
                body_contexts.push_back({});
            }
            body_contexts[i++] = BodyContext { traction_force, normal_force, link->WorldCoGPose().Pos()};
        }
        soil->post_update();
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void broadcast_soil(const std::shared_ptr<Soil>& soilPtr) {

        auto chunks = soilPtr->get_chunks()->get_active_chunks();

        for(auto& chunk : chunks) {
            hina_ssi_msgs::msgs::SoilChunk chunk_update_msg;

            auto field = chunk->container->get_field();
            auto x_w = field->x_vert_width;
            auto y_w = field->y_vert_width;

            for(uint32_t i = 0; i < x_w*y_w; i++) {
                auto update = chunk_update_msg.add_chunk_field();
                auto v3 = field->get_vertex_at_flattened_index(i)->v3;
                auto msg = gazebo::msgs::Vector3d();
                msg.set_x(v3.X());
                msg.set_y(v3.Y());
                msg.set_z(v3.Z());
                *update = msg;

                auto normal_msg = gazebo::msgs::Vector3d();
                auto normal = field->get_vertex_at_flattened_index(i)->v->normal;
                normal_msg.set_x(normal.X());
                normal_msg.set_y(normal.Y());
                normal_msg.set_z(normal.Z());

                auto update_normal = chunk_update_msg.add_normals_field();
                *update_normal = normal_msg;

                chunk_update_msg.add_footprint_field(field->get_vertex_at_flattened_index(i)->v->footprint);

            }

            chunk_update_msg.set_len_row(x_w);
            chunk_update_msg.set_len_col(y_w);
            chunk_update_msg.set_id_i(chunk->location.i);
            chunk_update_msg.set_id_j(chunk->location.j);
            chunk_update_msg.set_len_col(y_w);

            soilPub->Publish(chunk_update_msg);

            chunk->container->clear_footprint();
        }
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void broadcast_triangles(const std::vector<std::pair<TriangleContext,Footprint>>& context_v) {
        hina_ssi_msgs::msgs::Triangles triangle_update_msg;

        triangle_update_msg.set_len_triangles(context_v.size());

        for(auto& triangle : context_v) {
            auto centroid = triangle.first.tri.centroid();
            auto slip_vel = triangle.first.slip_velocity;
            auto force = triangle.second.force_x;
            auto normal = triangle.first.tri.normal().Normalize();
            auto t = triangle.second;
            auto contact = t.getSize() > 0;
            auto shear_displ = triangle.first.shear_displacement;

            auto update_centroids = triangle_update_msg.add_centroids();
            auto msg_centroids = gazebo::msgs::Vector3d();
            msg_centroids.set_x(centroid.X());
            msg_centroids.set_y(centroid.Y());
            msg_centroids.set_z(centroid.Z());
            *update_centroids = msg_centroids;

            auto update_slip = triangle_update_msg.add_slip_velocity();
            auto slip_msg = gazebo::msgs::Vector3d();
            slip_msg.set_x(slip_vel.X());
            slip_msg.set_y(slip_vel.Y());
            slip_msg.set_z(slip_vel.Z());
            *update_slip = slip_msg;

            auto update_forces = triangle_update_msg.add_forces();
            auto forces_msg = gazebo::msgs::Vector3d();
            forces_msg.set_x(force.X());
            forces_msg.set_y(force.Y());
            forces_msg.set_z(force.Z());
            *update_forces = forces_msg;

            auto update_normals = triangle_update_msg.add_normal();
            auto normals_msg = gazebo::msgs::Vector3d();
            normals_msg.set_x(normal.X());
            normals_msg.set_y(normal.Y());
            normals_msg.set_z(normal.Z());
            *update_normals = normals_msg;

            triangle_update_msg.add_shear_displacement(shear_displ);

            triangle_update_msg.add_contact(contact);


        }

        triPub->Publish(triangle_update_msg);

    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void broadcast_body(const std::vector<BodyContext>& context_v) {
            hina_ssi_msgs::msgs::BodyPhysics body_update_msg;

            body_update_msg.set_len(context_v.size());

            for(auto& body : context_v) {
                auto traction = body.traction;
                auto normal = body.normal;
                auto origin = body.origin;

                auto msg_traction = gazebo::msgs::Vector3d();
                msg_traction.set_x(traction.X());
                msg_traction.set_y(traction.Y());
                msg_traction.set_z(traction.Z());

                auto update_traction = body_update_msg.add_traction_force();
                *update_traction = msg_traction;

                auto msg_normal = gazebo::msgs::Vector3d();
                msg_normal.set_x(normal.X());
                msg_normal.set_y(normal.Y());
                msg_normal.set_z(normal.Z());

                auto update_normal = body_update_msg.add_normal_force();
                *update_normal = msg_normal;

                auto msg_origin = gazebo::msgs::Vector3d();
                msg_origin.set_x(origin.X());
                msg_origin.set_y(origin.Y());
                msg_origin.set_z(origin.Z());

                auto update_origin = body_update_msg.add_force_origin();
                *update_origin = msg_origin;

            }
            bodyPub->Publish(body_update_msg);
        }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    };
GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}
#endif

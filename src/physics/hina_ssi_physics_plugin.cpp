#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#define PHYS_PROFILER 0

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/common/Profiler.hh>
#include <memory>
#include <utility>

#include "soil/soil.h"
#include "dem/dem_loader.h"
#include "../common/field/uniform_field.h"

#include "SoilChunk.pb.h"

namespace hina {

class HinaSSIWorldPlugin : public WorldPlugin {

private:
    std::shared_ptr<Soil> p_soil = nullptr;
    std::unique_ptr<msgs::Vector3d[]> p_soil_v = nullptr;
    std::unique_ptr<msgs::Vector2d[]> p_soil_id_v = nullptr;

    transport::NodePtr node = nullptr;
    transport::PublisherPtr soilPub = nullptr;
    event::ConnectionPtr updateEventPtr = nullptr;
    event::ConnectionPtr onEntityAddedEventPtr = nullptr;
    physics::WorldPtr world = nullptr;
    sdf::ElementPtr sdf = nullptr;

    std::map<physics::LinkPtr, const common::Mesh *> mesh_lookup{};

    transport::Node* trspt = nullptr;

    common::Time time;
    double sec{};
    double last_sec{};
    double last_sec_viz{};
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

        time = common::Time::GetWallTime();
        sec = time.Double();

#ifdef PHYS_PROFILER
        IGN_PROFILE_BEGIN("TAROSCM::Soil Physics Update");
#endif
        double dt = sec - last_sec;
        double dt_viz = sec - last_sec_viz;

        update_soil(p_soil, dt);

#ifdef PHYS_PROFILER
        IGN_PROFILE_END();
        IGN_PROFILE_BEGIN("TAROSCM::Broadcast Soil");
#endif
        last_sec = sec;

        if (dt_viz > (1. / 1.f)) {
            broadcast_soil(p_soil);
            last_sec_viz = sec;
        }
#ifdef PHYS_PROFILER
        IGN_PROFILE_END();
#endif
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    void update_soil(const std::shared_ptr<Soil>& soil, double dt) {

        soil->pre_update();

        for (auto &iter: mesh_lookup) {
            auto link = iter.first;
            auto mesh = iter.second;

            for (uint32_t i = 0; i < mesh->GetSubMeshCount(); i++) {

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

                std::vector<std::vector<std::tuple<uint32_t, uint32_t,
                                        SoilChunk&, std::shared_ptr<FieldVertex<SoilVertex>>>>> footprint;
                std::vector<std::tuple<uint32_t, uint32_t,
                                        SoilChunk&, std::shared_ptr<FieldVertex<SoilVertex>>>> footprint_idx;

                double total_displaced_volume = 0.0f;
                float total_footprint_size = 0.0f;


                // Vertex level computations
#ifdef PHYS_PROFILER
                IGN_PROFILE_BEGIN("Collision Detection");
#endif
                #pragma omp parallel num_threads(col_threads) default(none) shared(std::cout,footprint, total_footprint_size, total_displaced_volume) firstprivate(footprint_idx, submesh, soil, crot, cpos, pos, rot, indices, dt, link)
                {
                    #pragma omp for nowait schedule(guided) reduction(+:total_displaced_volume)

                     for (uint32_t idx_unrolled = 0; idx_unrolled < (indices / 3); idx_unrolled++) {

                        auto idx = idx_unrolled * 3;

                        auto v0 = submesh->Vertex(submesh->GetIndex(idx));
                        auto v1 = submesh->Vertex(submesh->GetIndex(idx + 1));
                        auto v2 = submesh->Vertex(submesh->GetIndex(idx + 2));

                        auto cv0 = crot.RotateVector(v0) + cpos;
                        auto cv1 = crot.RotateVector(v1) + cpos;
                        auto cv2 = crot.RotateVector(v2) + cpos;

                        auto c1v0 = rot.RotateVector(cv0) + pos;
                        auto c1v1 = rot.RotateVector(cv1) + pos;
                        auto c1v2 = rot.RotateVector(cv2) + pos;

                        auto meshTri = Triangle(c1v0, c1v1, c1v2);

                        double displaced_volume = 0.0f;

                        footprint_idx = this->p_soil->try_deform(meshTri, link, displaced_volume);

                        total_displaced_volume += displaced_volume;

                        #pragma omp critical
                        {
                            footprint.push_back(footprint_idx);
                        }

                     }
                }
#ifdef PHYS_PROFILER
                IGN_PROFILE_END();
#endif


                //this->p_soil->compute_footprint_stage(footprint, total_displaced_volume);
            }
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

    };
GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}
#endif

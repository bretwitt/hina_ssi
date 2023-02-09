#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include "../common/field/uniform_field.h"
#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <utility>
#include "soil/soil.h"
#include "dem/dem_loader.h"
#include "Soil.pb.h"
#include "soil/soil_chunk_location_metadata.h"

namespace hina {
    class HinaSSIWorldPlugin : public WorldPlugin {

    private:
        std::shared_ptr<Soil> soilPtr = nullptr;
        std::unique_ptr<msgs::Vector3d[]> soil_v = nullptr;
        std::unique_ptr<msgs::Vector2d[]> soil_id_v = nullptr;

        transport::NodePtr node = nullptr;
        transport::PublisherPtr soilPub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;
        event::ConnectionPtr onEntityAddedEventPtr = nullptr;
        physics::WorldPtr world = nullptr;
        sdf::ElementPtr sdf = nullptr;

        Vector3d v3;
        std::map<physics::LinkPtr, const common::Mesh *> mesh_lookup{};

        common::Time time;
        double sec{};
        double last_sec{};
        double last_sec_viz{};
        int col_threads = 3;

    public:

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
            updateEventPtr = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&HinaSSIWorldPlugin::update, this));
            onEntityAddedEventPtr = event::Events::ConnectAddEntity(
                    boost::bind(&HinaSSIWorldPlugin::OnEntityAdded, this, _1));
            world = _world;
            sdf = _sdf;
            init_threading();
            init_soil();
            init_transport();
            init_models();
        }

        void init_threading() {
            if (sdf->HasElement("col_threads")) {
                col_threads = sdf->GetElement("col_threads")->Get<int>();
            }
        }

        void OnEntityAdded(const std::string &str) {
            init_links(str);
        }

        void init_models() {
            auto model_v = world->Models();
            for (const auto &model: model_v) {
                init_links(model->GetName());
            }
        }

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
                            }
                        }
                    }
                }
            }
        }

        void init_soil() {
            if (sdf->HasElement("dem")) {
                init_dem();
            } else if(sdf->HasElement("sandbox")){
                init_sandbox();
            }

            auto params = sdf->GetElement("params");

            auto k_phi = params->GetElement("k_phi")->Get<double>();
            auto k_e = params->GetElement("k_e")->Get<double>();
            auto c = params->GetElement("c")->Get<double>();
            auto phi = params->GetElement("phi")->Get<double>();

//            auto f = soilPtr->get_chunk()->field;
//
//            for(uint32_t i = 0; i < f->x_vert_width*f->y_vert_width; i++) {
//                auto vtx = f->get_vertex_at_flattened_index(i)->v;
//                vtx->k_phi = k_phi;
//                vtx->k_e = k_e;
//                vtx->c = c;
//                vtx->phi = phi;
//            }
        }

        void init_sandbox() {
            auto sandbox_elem = sdf->GetElement("sandbox");
            int x_width = sandbox_elem->GetElement("width_x")->Get<int>();
            int y_width = sandbox_elem->GetElement("width_y")->Get<int>();
            auto scale = sandbox_elem->GetElement("resolution")->Get<double>();
            auto angle = sandbox_elem->GetElement("angle")->Get<double>();

            soilPtr = std::make_shared<Soil>(SandboxConfig{x_width, y_width, scale, angle});
        }

        void init_dem() {
            auto dem_elem = sdf->GetElement("dem");

            auto filename = dem_elem->GetElement("file")->Get<std::string>();
            std::string file_name = gazebo::common::SystemPaths::Instance()->FindFile(filename);
            auto dem = DEMLoader::load_dem_from_geotiff(file_name);

            if(dem_elem->HasElement("upscale_res")) {
                auto upscale_res = dem_elem->GetElement("upscale_res")->Get<double>();
                dem->upsample(upscale_res);
            }
            soilPtr = std::make_shared<Soil>(dem);
        }

        void init_transport() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            soilPub = node->Advertise<hina_ssi_msgs::msgs::Soil>("~/soil");
        }

        void load_mesh(const physics::LinkPtr &link, const std::string &mesh_uri) {
            auto mesh = common::MeshManager::Instance()->Load(mesh_uri);
            mesh_lookup.insert(std::pair<physics::LinkPtr, const common::Mesh *>(link, mesh));
        }

        void update() {
            time = common::Time::GetWallTime();
            sec = time.Double();

            double dt = sec - last_sec;
            double dt_viz = sec - last_sec_viz;

            update_soil(soilPtr, dt);

            last_sec = sec;

            if (dt_viz > (1. / 4.f)) {
                broadcast_soil(soilPtr);
                last_sec_viz = sec;
            }
        }

        void update_soil(std::shared_ptr<Soil> soil, float dt) {

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

                    soil->query_chunk(max);
                    soil->query_chunk(min);         // TODO: Loading radius

                    std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> footprint;
                    std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> footprint_idx;
                    float total_displaced_volume = 0.0f;

                    #pragma omp parallel num_threads(col_threads) default(none) shared(footprint, total_displaced_volume) firstprivate(footprint_idx, submesh, soil, crot, cpos, pos, rot, indices, dt, link)
                    {
                    #pragma omp for nowait schedule(guided) //reduction(+:total_displaced_volume)
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

                            float displaced_volume = 0.0f;
                            footprint_idx = soil->try_deform(meshTri, link, dt, displaced_volume);
                            total_displaced_volume += displaced_volume;
                        }
                    }
                }
            }
            soil->post_update();
        }

        void broadcast_soil(std::shared_ptr<Soil> soilPtr) {
            hina_ssi_msgs::msgs::Soil soilMsg;

            auto chunk = soilPtr->chunks.get_active_chunks();

            if(chunk.size() <= 0) {
                return;
            }

            soilMsg.set_len_col(2);
            soilMsg.set_len_col(2);
            soil_v = std::make_unique<msgs::Vector3d[]>(4);

            auto v = soilMsg.add_chunk_field();
            for(uint32_t i = 0; i < 2; i++) {
                for(uint32_t j = 0; j < 2; j++) {
                    soil_v[i*j] = msgs::Vector3d();
                    soil_v[i*j].set_x(0);
                    soil_v[i*j].set_y(0);
                    soil_v[i*j].set_z(0);
                    *v = soil_v[i];
                }
            }

            auto id_v = std::make_unique<msgs::Vector2d[]>(1);
            auto v1 = soilMsg.add_id_field();
            id_v[0] = msgs::Vector2d();
            id_v[0].set_x(0);
            id_v[0].set_y(0);
            *v1 = id_v[0];
            soilPub->Publish(soilMsg);

//            if (soil_v == nullptr){
//                soil_v = std::make_unique<msgs::Vector3d[]>(chunk.size()*field->x_vert_width * field->y_vert_width);
//                soil_id_v = std::make_unique<msgs::Vector2d[]>(chunk.size());
//            }

//            auto x_w = field->x_vert_width;
//            auto y_w = field->y_vert_width;
//
//            uint32_t counter = 0;
//            for(auto& c : chunk) {
//                Vector3d vert;
//                Vector2d id;
//                soil_id_v[counter] = msgs::Vector2d();
//                auto sc = c->location;
//                soil_id_v[counter].set_x(sc.i);
//                soil_id_v[counter].set_y(sc.j);
//                for (int idx = 0; idx < x_w * y_w; idx++) {
//                    vert = c->container->field->vertex_at_flattened_index(idx)->v3;
//                    (soil_v)[idx+(counter*x_w*y_w)] = msgs::Vector3d();
//                    (soil_v)[idx+(counter*x_w*y_w)].set_x(vert.X());
//                    (soil_v)[idx+(counter*x_w*y_w)].set_y(vert.Y());
//                    (soil_v)[idx+(counter*x_w*y_w)].set_z(vert.Z());
//                }
//                counter++;
//            }
//            soilMsg.set_len_col(x_w);
//            soilMsg.set_len_row(y_w);
//
//            for (uint32_t i = 0; i < chunk.size()*x_w * y_w; i++) {
//                auto v = soilMsg.add_chunk_field();
//                *v = soil_v[i];
//            }
//            for (uint32_t i = 0; i < chunk.size(); i++) {
//                auto v = soilMsg.add_id_field();
//                *v = soil_id_v[i];
//            }

            //soilPub->Publish(soilMsg);
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}
#endif

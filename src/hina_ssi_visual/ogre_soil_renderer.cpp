#ifndef MESH_GENERATOR_CPP
#define MESH_GENERATOR_CPP

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>

#include <ignition/math.hh>
#include <utility>
#include <gazebo/rendering/ogre_gazebo.h>
#include "../common/soil/soil.h"

using ignition::math::Vector2d;

namespace gazebo {
    class OgreSoilRenderer {
    private:
        rendering::ScenePtr scenePtr = nullptr;
        Ogre::ManualObject* manObj;

        void tri_update(std::shared_ptr<Soil> soil, Ogre::ManualObject* manObj, uint32_t x_size, uint32_t y_size) {
            uint32_t nVerts = x_size*y_size;
//
//            bool do_calculate_normals = render_config::getInstance()->render_mode == render_config::SoilRenderMode::SOLID;
//
//            if(do_calculate_normals) {
//                if(vertex_normals == nullptr) vertex_normals = new Ogre::Vector3[nVerts];
//                calculate_vertex_normals(soil, x_size, y_size, vertex_normals);
//            }

            for(uint32_t i = 0; i < nVerts; i++) {
                auto vertex = soil->get_data()->vertex_at_flattened_index(i);
                auto v3 = vertex->v3;

                manObj->position(v3.X(), v3.Y(), v3.Z());
                manObj->colour(120, 120, 120);

            }
            //auto indices = std::move(soil->get_data()->indices);

            for(uint32_t i = 0; i < (x_size - 1)*(y_size - 1)*3*2;) {
                manObj->index(soil->get_data()->indices[i++]);
            }
        }

        void calculate_vertex_normals(std::shared_ptr<Soil> soil, uint32_t x_size, uint32_t y_size, Ogre::Vector3* vertex_normals) {

            std::fill_n(vertex_normals, x_size*y_size, Ogre::Vector3(0,0,0));

            for(uint32_t y = 0; y < y_size - 1; y++) {
                for(uint32_t x = 0; x < x_size - 1; x++) {
                    uint32_t a = (x_size * x) + y;
                    uint32_t b = (x_size * (x + 1)) + y;
                    uint32_t c = (x_size * (x + 1)) + (y + 1);
                    uint32_t d = (x_size * x) + (y + 1);

                    auto vA = soil->get_data()->vertex_at_flattened_index(a)->v3;
                    auto vB = soil->get_data()->vertex_at_flattened_index(b)->v3;
                    auto vC = soil->get_data()->vertex_at_flattened_index(c)->v3;
                    auto vD = soil->get_data()->vertex_at_flattened_index(d)->v3;

                    auto av0 = Ogre::Vector3(vA.X(), vA.Y(), vA.Z() );
                    auto av1 = Ogre::Vector3(vD.X(), vD.Y(), vD.Z() );
                    auto av2 = Ogre::Vector3(vC.X(), vC.Y(), vC.Z() );

                    auto bv0 = Ogre::Vector3(vC.X(), vC.Y(), vC.Z() );
                    auto bv1 = Ogre::Vector3(vB.X(), vB.Y(), vB.Z() );
                    auto bv2 = Ogre::Vector3(vA.X(), vA.Y(), vA.Z() );

                    Ogre::Vector3 aNormal = Ogre::Math::calculateBasicFaceNormal(av0, av1, av2);
                    Ogre::Vector3 bNormal = Ogre::Math::calculateBasicFaceNormal(bv0, bv1, bv2);

                    vertex_normals[a] = (vertex_normals[a] + aNormal + bNormal).normalisedCopy();
                    vertex_normals[b] = (vertex_normals[b] + bNormal).normalisedCopy();
                    vertex_normals[c] = (vertex_normals[c] + aNormal + bNormal).normalisedCopy();
                    vertex_normals[d] = (vertex_normals[d] + aNormal).normalisedCopy();
                }
            }
        }
    public:
        void setScenePtr(rendering::ScenePtr scenePtr) {
            this->scenePtr = std::move(scenePtr);
        }

        void create_ogre_mesh(std::shared_ptr<Soil> soil) {
            auto sceneManager = scenePtr->OgreSceneManager();

            manObj = sceneManager->createManualObject("terrain_mesh");

            uint32_t x_size = soil->get_data()->x_width;
            uint32_t y_size = soil->get_data()->y_width;

            manObj->begin("Hina/Soil", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            tri_update(soil, manObj, x_size, y_size);
            manObj->end();

            sceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(manObj);
        }

        void update_ogre_mesh(std::shared_ptr<Soil> soil) {
            uint32_t x_size = soil->get_data()->x_width;
            uint32_t y_size = soil->get_data()->y_width;

            manObj->beginUpdate(0);
            tri_update(soil, manObj, x_size, y_size);
            manObj->end();
        }
    };
}

#endif

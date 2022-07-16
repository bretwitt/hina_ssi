#ifndef MESH_GENERATOR_CPP
#define MESH_GENERATOR_CPP

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>

#include <ignition/math.hh>
#include <utility>
#include <gazebo/rendering/ogre_gazebo.h>
#include "../soil.cpp"

using ignition::math::Vector2d;

namespace gazebo {
    class MeshGenerator {
    private:
        rendering::ScenePtr scenePtr = nullptr;
        Vector3d* field = nullptr;
        Ogre::Vector3* vertex_normals;
        Ogre::ManualObject* manObj;

        void tri_update(Soil* soil, Ogre::ManualObject* manObj, uint32_t x_size, uint32_t y_size) {
            uint32_t nVerts = x_size*y_size;

            field = soil->get_data().soil_field;
            if(vertex_normals == nullptr) vertex_normals = new Ogre::Vector3[nVerts];

            calculate_vertex_normals(field, x_size, y_size, vertex_normals);

            auto vertices = soil->get_data().soil_field;
            for(uint32_t i = 0; i < nVerts; i++) {
                auto v3 = vertices[i];
                manObj->position(v3.X(),v3.Y(),v3.Z());
                manObj->normal(vertex_normals[i]);

                if(i == 0) manObj->textureCoord(0,0);
                if(i == nVerts - 1) manObj->textureCoord(1,1);
            }

            auto indices = soil->get_data().indices;
            for(uint32_t i = 0; i < (x_size - 1)*(y_size - 1)*3*2;) {
                manObj->index(indices[i++]);
            }
        }

        void calculate_vertex_normals(Vector3d* field, uint32_t x_size, uint32_t y_size, Ogre::Vector3* vertex_normals) {

            std::fill_n(vertex_normals, x_size*y_size, Ogre::Vector3(0,0,0));

            for(uint32_t y = 0; y < y_size - 1; y++) {
                for(uint32_t x = 0; x < x_size - 1; x++) {
                    uint32_t a = (x_size * x) + y;
                    uint32_t b = (x_size * (x + 1)) + y;
                    uint32_t c = (x_size * (x + 1)) + (y + 1);
                    uint32_t d = (x_size * x) + (y + 1);

                    auto vA = field[a];
                    auto vB = field[b];
                    auto vC = field[c];
                    auto vD = field[d];

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

        void create_ogre_mesh(Soil* soil) {
            auto sceneManager = scenePtr->OgreSceneManager();
            manObj = sceneManager->createManualObject("terrain_mesh");

            uint32_t x_size = soil->get_data().x_width;
            uint32_t y_size = soil->get_data().y_width;

            manObj->begin("Hina/Soil", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            tri_update(soil, manObj, x_size, y_size);
            manObj->end();
            sceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(manObj);
        }

        void update_ogre_mesh(Soil* soil) {
            uint32_t x_size = soil->get_data().x_width;
            uint32_t y_size = soil->get_data().y_width;

            manObj->beginUpdate(0);
            tri_update(soil, manObj, x_size, y_size);
            manObj->end();
        }
    };
}

#endif

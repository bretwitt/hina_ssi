#ifndef MESH_GENERATOR_CPP
#define MESH_GENERATOR_CPP

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>

#include <ignition/math.hh>
#include <gazebo/rendering/ogre_gazebo.h>
#include "../soil.cpp"

namespace gazebo {
    class MeshGenerator {
    private:
        rendering::ScenePtr scenePtr = nullptr;

        void tri_update(Soil* soil, Ogre::ManualObject* manObj, uint32_t x_size, uint32_t y_size) {
            uint32_t nVerts = x_size*y_size;

            for(uint32_t i = 0; i < nVerts; i++) {
                auto v3 = soil->get_data().soil_field[i];
                manObj->position(v3.X(),v3.Y(),v3.Z());
            }

            for(uint32_t y = 0; y < y_size - 1; y++) {
                for(uint32_t x = 0; x < x_size - 1; x++) {
                    uint32_t a = (x_size * x) + y;
                    uint32_t b = (x_size * (x + 1)) + y;
                    uint32_t c = (x_size * (x + 1)) + (y + 1);
                    uint32_t d = (x_size * x) + (y + 1);

                    manObj->index(a);
                    manObj->index(d);
                    manObj->index(c);

                    manObj->index(c);
                    manObj->index(b);
                    manObj->index(a);
                }
            }
        }
    public:
        void setScenePtr(rendering::ScenePtr scenePtr) {
            this->scenePtr = scenePtr;
        }

        void create_ogre_mesh(Soil* soil) {
            auto sceneManager = scenePtr->OgreSceneManager();
            auto* manObj = sceneManager->createManualObject("terrain_mesh");

            uint32_t x_size = soil->get_data().x_width;
            uint32_t y_size = soil->get_data().y_width;

            manObj->begin("", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            tri_update(soil, manObj, x_size, y_size);
            manObj->end();

            sceneManager->getRootSceneNode()->createChildSceneNode()->attachObject(manObj);
        }

        void update_ogre_mesh(Soil* soil) {
            auto sceneManager = scenePtr->OgreSceneManager();
            auto* manObj = sceneManager->getManualObject("terrain_mesh");
            uint32_t x_size = soil->get_data().x_width;
            uint32_t y_size = soil->get_data().y_width;

            manObj->beginUpdate(0);
            tri_update(soil, manObj, x_size, y_size);
            manObj->end();
        }
    };
}

#endif

#ifndef MESH_GENERATOR_CPP
#define MESH_GENERATOR_CPP

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>

#include <ignition/math.hh>
#include <utility>
#include <gazebo/rendering/ogre_gazebo.h>
#include "height_attributes.h"
#include "../common/field/uniform_field.h"

using ignition::math::Vector2d;

using namespace gazebo;

namespace hina {
    class OgreSoilRenderer {
    private:
        rendering::ScenePtr scenePtr = nullptr;
        Ogre::ManualObject *manObj = nullptr;
        Ogre::SceneNode *child = nullptr;

        void
        tri_update(const std::shared_ptr<UniformField<ColorAttributes>> &field, Ogre::ManualObject *manObj, uint32_t x_size, uint32_t y_size) {

            uint32_t nVerts = x_size * y_size;

            for (uint32_t i = 0; i < nVerts; i++) {
                auto vertex = field->vertex_at_flattened_index(i);
                auto v3 = vertex->v3;

                manObj->position(v3.X(), v3.Y(), v3.Z());
                manObj->colour(vertex->v->r, vertex->v->g, vertex->v->b);
            }

            for (uint32_t i = 0; i < (x_size - 1) * (y_size - 1) * 3 * 2;) {
                manObj->index(field->indices[i++]);
            }
        }

    public:
        void setScenePtr(rendering::ScenePtr scenePtr) {
            this->scenePtr = std::move(scenePtr);
        }


        void create_ogre_mesh(const std::shared_ptr<UniformField<ColorAttributes>> &field, std::string id) {
            auto sceneManager = scenePtr->OgreSceneManager();

            manObj = sceneManager->createManualObject("terrain_mesh_" + id);

            uint32_t x_size = field->x_vert_width;
            uint32_t y_size = field->y_vert_width;

            manObj->begin("Hina/Soil", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            tri_update(field, manObj, x_size, y_size);
            manObj->end();

            child = sceneManager->getRootSceneNode()->createChildSceneNode();
            child->attachObject(manObj);

        }


        void update_ogre_mesh(const std::shared_ptr<UniformField<ColorAttributes>> &field) {
            uint32_t x_size = field->x_vert_width;
            uint32_t y_size = field->y_vert_width;

            manObj->beginUpdate(0);
            tri_update(field, manObj, x_size, y_size);
            manObj->end();
        }

        void delete_ogre_mesh() {
            if(manObj != nullptr) {
                auto sceneManager = scenePtr->OgreSceneManager();
                sceneManager->getRootSceneNode()->removeChild(child);
                sceneManager->destroyManualObject(manObj);
            }
        }
    };
}
#endif

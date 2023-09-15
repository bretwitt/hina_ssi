#ifndef BODY_INFO_RENDER_CPP
#define BODY_INFO_RENDER_CPP

#include <gazebo/rendering/rendering.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include "visual_body_physics.h"

using namespace ignition::math;
using namespace gazebo;

namespace hina {
    class BodyInfoRenderer {
        VisualBodyPhysics _physics;
        gazebo::rendering::DynamicLines _lines;
        Ogre::ManualObject *line = nullptr;
        Ogre::SceneNode *child = nullptr;
        rendering::ScenePtr scenePtr = nullptr;
        rendering::VisualPtr _visual = nullptr;
//        std::vector<Ogre::ManualObject*> lines;

        bool init_viz = 0;

    public:

        ~BodyInfoRenderer() {
            if(line != nullptr) {
                auto sceneManager = scenePtr->OgreSceneManager();
                sceneManager->getRootSceneNode()->removeChild(child);
                sceneManager->destroyManualObject(line);
            }

        }
        void init(rendering::VisualPtr visual) {
            _visual = visual;
        }

        void update_body(const VisualBodyPhysics& physics) {
            _physics = physics;

            int size = _physics.origin.size();

            if(size == 0) {
                return;
            }

            if(!init_viz) {
                std::cout <<"Initializing Body Visualization " << size << std::endl;
                init_buffers(size);
                init_viz = true;
                std::cout << "Finished initializing Body Visualization" << std::endl;
                return;
            }

            line->clear();
            line->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

            for(int i = 0; i < size; i++) {
                auto origin = _physics.origin[i];
                auto force = _physics.normal[i];

                line->colour(1.0,0,0,1.0);
                line->position(origin.x(),origin.y(),origin.z());
                line->position(force.x()+origin.x(),force.y()+origin.y(),force.z()+origin.z());

            }
            line->end();
        }

        void init_buffers(int triangles) {

            Ogre::Vector3 start(0, 0, 0);
            Ogre::Vector3 end(1, 0, 1);
            Ogre::SceneManager* sceneMgr = _visual->GetScene()->OgreSceneManager();

            line = sceneMgr->createManualObject("body_physics");
            line->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);
            line->position(start);
            line->position(start);
            line->end();

            child = sceneMgr->getRootSceneNode()->createChildSceneNode();
            child->attachObject(line);

        }

    };
}

#endif
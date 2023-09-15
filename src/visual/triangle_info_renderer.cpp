#ifndef TRIANGLE_INFO_RENDER_CPP
#define TRIANGLE_INFO_RENDER_CPP

#include <gazebo/rendering/rendering.hh>
#include "visual_triangles.h"
#include <gazebo/common/common.hh>
#include <ignition/math.hh>

using namespace ignition::math;
using namespace gazebo;

namespace hina {
    class TriangleInfoRenderer {
        VisualTriangles _triangles;
        gazebo::rendering::DynamicLines _lines;
        Ogre::ManualObject *line = nullptr;
        Ogre::SceneNode *child = nullptr;
        rendering::ScenePtr scenePtr = nullptr;
        rendering::VisualPtr _visual = nullptr;
//        std::vector<Ogre::ManualObject*> lines;

        bool init_viz = 0;

    public:

        ~TriangleInfoRenderer() {
            if(line != nullptr) {
                auto sceneManager = scenePtr->OgreSceneManager();
                sceneManager->getRootSceneNode()->removeChild(child);
                sceneManager->destroyManualObject(line);
            }

        }
        void init(rendering::VisualPtr visual) {
            _visual = visual;
        }

        void update_triangles(const VisualTriangles& triangles) {
            _triangles = triangles;

            int size = _triangles.centroid.size();

            if(size == 0) {
                return;
            }

            if(!init_viz) {
                std::cout <<"Initializing " << size << std::endl;
                init_buffers(size);
                init_viz = true;
                std::cout << "Finished initializing" << std::endl;
                return;
            }

            line->clear();
            line->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST);

            for(int i = 0; i < size; i++) {
                auto center = _triangles.centroid[i];
                auto force = _triangles.forces[i];
                auto normal = _triangles.normal[i];
                auto slip_velocity = _triangles.slip_velocity[i];
                auto contact = _triangles.contact[i];
                auto shear_displ = _triangles.shear_displacement[i];
                if(contact) {
//                    Normal
//                    line->position(center.x(),center.y(),center.z());
//                    line->position(normal.x()+center.x(),normal.y()+center.y(),normal.z()+center.z());

//                     Force
                    line->position(center.x(),center.y(),center.z());
                    line->position(force.x()+center.x(),force.y()+center.y(),force.z()+center.z());

//                    Shear Displacement
//                    line->position(center.x(),center.y(),center.z());
//                    line->position(shear_displ*normal.x()+center.x(),shear_displ*normal.y()+center.y(),shear_displ*normal.z()+center.z());

//                    Slip Velocity
//                    line->position(center.x(),center.y(),center.z());
//                    line->position(slip_velocity.x()+center.x(),slip_velocity.y()+center.y(),slip_velocity.z()+center.z());
                }
            }
            line->end();
        }

        void init_buffers(int triangles) {

            Ogre::Vector3 start(0, 0, 0);
            Ogre::Vector3 end(1, 0, 1);
            Ogre::SceneManager* sceneMgr = _visual->GetScene()->OgreSceneManager();

            line = sceneMgr->createManualObject("triangles");
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
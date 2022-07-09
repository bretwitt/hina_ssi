#ifndef MESH_GENERATOR_CPP
#define MESH_GENERATOR_CPP

#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include "../soil.cpp"

namespace gazebo {
    class MeshGenerator {
    private:
        common::Mesh* mesh = nullptr;
        common::SubMesh* terrainSubMesh = nullptr;
    public:
        ~MeshGenerator() {
            delete mesh;
            delete terrainSubMesh;
        }

        common::Mesh* generate_mesh(Soil* soil) {
            auto mesh = new common::Mesh();
            auto terrainSubMesh = new common::SubMesh();

            terrainSubMesh->SetPrimitiveType(common::SubMesh::PrimitiveType::TRIANGLES);

            int x_size = soil->get_data().x_width;
            int y_size = soil->get_data().y_width;

            for(int y = 0; y < y_size; y++) {
                for (int x = 0; x < x_size; x++) {
                    terrainSubMesh->AddVertex(soil->get_data().soil_field[x][y]);
                }
            }

            for(int y = 0; y < y_size - 1; y++) {
                for(int x = 0; x < x_size - 1; x++) {
                    int a = (x_size * x) + y;
                    int b = (x_size * (x + 1)) + y;
                    int c = (x_size * (x + 1)) + (y + 1);
                    int d = (x_size * x) + (y + 1);
                    terrainSubMesh->AddIndex(a);
                    terrainSubMesh->AddIndex(d);
                    terrainSubMesh->AddIndex(c);

                    terrainSubMesh->AddIndex(c);
                    terrainSubMesh->AddIndex(b);
                    terrainSubMesh->AddIndex(a);
                }
            }

            terrainSubMesh->RecalculateNormals();

            mesh->AddSubMesh(terrainSubMesh);
            mesh->SetName("terrain_mesh");

            auto material = new common::Material(ignition::math::Color(0,0,0));
            material->SetTextureImage("terrain.png");

            mesh->AddMaterial(material);

            return mesh;
        }
    };
}

#endif

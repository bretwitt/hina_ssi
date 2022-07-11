#ifndef MESH_GENERATOR_CPP
#define MESH_GENERATOR_CPP

#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include "../soil.cpp"

namespace gazebo {
    class MeshGenerator {
    private:
        common::SubMesh* terrainSubMesh = nullptr;
        common::Mesh* mesh = nullptr;

    public:
        ~MeshGenerator() {
            delete mesh;
            delete terrainSubMesh;
        }

        common::Mesh* generate_mesh(Soil* soil) {
            mesh = new common::Mesh();

            generate_submesh(soil);

            mesh->AddSubMesh(terrainSubMesh);
            mesh->SetName("terrain_mesh");

            return mesh;
        }

        void generate_submesh(Soil* soil) {
            terrainSubMesh = new common::SubMesh();
            terrainSubMesh->SetName("terrain_submesh");

            int x_size = soil->get_data().x_width;
            int y_size = soil->get_data().y_width;

            terrainSubMesh->SetPrimitiveType(common::SubMesh::PrimitiveType::TRIANGLES);

            for(int i = 0; i < x_size*y_size; i++) {
                auto vert = soil->get_data().soil_field[i];
                terrainSubMesh->AddVertex(vert);
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
        }

        void update_submesh(Soil* soil) {
            int x_size = soil->get_data().x_width;
            int y_size = soil->get_data().y_width;

            for(int i = 0; i < x_size*y_size; i++) {
                auto vert = soil->get_data().soil_field[i];
                terrainSubMesh->SetVertex(i,Vector3d(vert.X(),vert.Y(), 0));
            }
            terrainSubMesh->RecalculateNormals();
        }

    };
}

#endif

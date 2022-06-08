#include <gazebo/common/common.hh>
#include <ignition/math.hh>

namespace gazebo {
    class MeshGenerator {
    public:
        common::Mesh* generate_mesh() {
            auto mesh = new common::Mesh();
            auto terrainSubMesh = new common::SubMesh();
            auto *material = new common::Material(ignition::math::Color::White);

            terrainSubMesh->SetPrimitiveType(common::SubMesh::PrimitiveType::TRIANGLES);

            int size = 3;
            auto* vertices = new float[size*size*3];
            int* indices = new int[(size*size - 1)*3];

            float x = 0;
            float y = 0;
            int ind = 0;

            for(int i = 0; i < size*size*3; i+=3) {
                vertices[i] = x;
                vertices[i + 1] = y;
                vertices[i + 2] = 1.0f;

                if((ind + 1) % size == 0) {
                    y = 0;
                    x++;
                }
                else {
                    y++;
                }
                ind++;
            }

            int n = 0;
            int q = 0;

            for(int i = 0; i < (size*size - 1) / 2; i++) {
                int n0 = n;
                int n1 = n + 1;
                int n2 = n + size;
                int n3 = n + size + 1;

                indices[q] = n0;
                indices[q + 1] = n3;
                indices[q + 2] = n1;
                indices[q + 3] = n0;
                indices[q + 4] = n2;
                indices[q + 5] = n3;

                q += 6;
            }

            for(int q = 0; q < (size*size - 1) * 3; q+=6) {
                std::cout << indices[q] << " " << indices[q+1] << " " << indices[q+2] << " " << indices[q+3] << " " << indices[q+4] << " " << indices[q+5] << " " << std::endl;
            }

            terrainSubMesh->SetVertexCount(size*size);
            terrainSubMesh->SetIndexCount((size*size - 1) / 2);
            terrainSubMesh->FillArrays(&vertices, &indices);
            terrainSubMesh->RecalculateNormals();

            mesh->AddSubMesh(terrainSubMesh);
            mesh->AddMaterial(material);

            return mesh;
        }
    };
}

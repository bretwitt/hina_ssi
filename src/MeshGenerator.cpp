#include <gazebo/common/common.hh>

namespace gazebo {
    class MeshGenerator {
    public:
        common::Mesh* generate_mesh() {
            auto mesh = new common::Mesh();
            auto terrainSubMesh = new common::SubMesh();
            terrainSubMesh->SetPrimitiveType(common::SubMesh::PrimitiveType::TRIANGLES);

            int size = 3;
            float** vertices = new float*[size*size*3];
            int** indices = new int*[(size*size - 1)*3];

            float x = 0;
            float y = 0;

            for(int i = 0; i < size*size*3; i++) {
                if(i % size == 0) {
                    y = 0;
                    x++;
                }
                else if(i % size != 0) {
                    y++;
                }
                if(i % 3 == 0) {
                    vertices[i] = new float(x);
                }
                else if(i % 3 == 1) {
                    vertices[i] = new float(y);
                }
                else if(i % 3 == 2) {
                    vertices[i] = new float(0.0f);
                }
            }

            int n = 0;
            int q = 0;
            for(int i = 0; i < (size*size - 1) / 2; i++) {
                int n0 = n;
                int n1 = n + 1;
                int n2 = n + size + 1;
                int n3 = n + size + 2;

                indices[q] = new int(n0);
                indices[q + 1] = new int(n1);
                indices[q + 2] = new int(n3);
                indices[q + 3] = new int(n0);
                indices[q + 4] = new int(n2);
                indices[q + 5] = new int(n3);

                n++;
                q += 6;
            }

            terrainSubMesh->SetVertexCount(size*size);
            terrainSubMesh->SetIndexCount((size*size - 1) / 2);
            terrainSubMesh->FillArrays(vertices, indices);
            terrainSubMesh->RecalculateNormals();

            mesh->AddSubMesh(terrainSubMesh);

            return mesh;
        }
    };
}

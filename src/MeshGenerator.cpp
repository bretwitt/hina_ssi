#include <gazebo/common/common.hh>
#include <ignition/math.hh>

namespace gazebo {
    class MeshGenerator {
    public:
        common::Mesh* generate_mesh() {
            auto mesh = new common::Mesh();
            auto terrainSubMesh = new common::SubMesh();

            terrainSubMesh->SetPrimitiveType(common::SubMesh::PrimitiveType::TRIANGLES);
            terrainSubMesh->AddVertex(0.0,0.0,5.0);
            terrainSubMesh->AddVertex(5.0,5.0,5.0);
            terrainSubMesh->AddVertex(0.0,5.0,5.0);
            terrainSubMesh->AddVertex(0.0,2.0,5.0);
            terrainSubMesh->AddIndex(0);
            terrainSubMesh->AddIndex(1);
            terrainSubMesh->AddIndex(2);
            terrainSubMesh->AddIndex(3);
            terrainSubMesh->AddIndex(4);
            terrainSubMesh->RecalculateNormals();

            mesh->AddSubMesh(terrainSubMesh);
            mesh->SetName("terrain_mesh");
            return mesh;
        }
    };
}

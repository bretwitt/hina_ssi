#ifndef HINA_SSI_FOOTPRINT_H
#define HINA_SSI_FOOTPRINT_H

#include <vector>
#include <memory>
#include <cstdint>
#include "../../common/field/field_vertex.h"
#include "soil_vertex.h"

namespace hina {
    struct Contact {
        uint32_t x; // x in chunk
        uint32_t y; // y in chunk
        int i;      // Soil chunk i
        int j;      // Soil chunk j
        std::shared_ptr<FieldVertex<SoilVertex>> vtx; // vertex
    };
    class Footprint {
    public:
        std::vector<Contact> footprint;
        int getSize() {
            return footprint.size();
        }
    };
}

#endif //HINA_SSI_FOOTPRINT_H

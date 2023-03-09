
#ifndef HINA_SSI_PLUGIN_SOIL_CHUNK_LOCATION_METADATA_H
#define HINA_SSI_PLUGIN_SOIL_CHUNK_LOCATION_METADATA_H

namespace hina {
    struct SoilChunkLocationMetadata {
        int i;
        int j;
        Vector2d origin; // Vector2d of first vertex
    };
};

#endif //HINA_SSI_PLUGIN_SOIL_CHUNK_LOCATION_METADATA_H

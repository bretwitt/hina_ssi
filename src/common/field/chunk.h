#ifndef HINA_SSI_PLUGIN_CHUNK_H
#define HINA_SSI_PLUGIN_CHUNK_H

#include "chunked_field_location.h"

template<class T>
struct Chunk {
    ChunkedFieldLocation location;
    T container;
    bool keep_loaded_flag = false;
    Chunk(T container, ChunkedFieldLocation location) {
        this->container = container;
        this->location = location;
    }
};

#endif //HINA_SSI_PLUGIN_CHUNK_H

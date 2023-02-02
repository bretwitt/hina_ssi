#ifndef HINA_SSI_PLUGIN_CHUNK_FIELD_H
#define HINA_SSI_PLUGIN_CHUNK_FIELD_H

#include "uniform_field.h"
#include <gazebo/common/common.hh>

using ignition::math::Vector2d;

template<class T>
class ChunkedField {

    struct ChunkedFieldLocation {
        int i;
        int j;
        Vector2d origin;
    };

    struct Chunk {
        ChunkedFieldLocation location;
        T container;
        bool keep_loaded_flag;
        Chunk(T container, ChunkedFieldLocation location) {
            this->container = container;
            this->location = location;
        }
    };

private:
    std::unordered_map<int,std::unordered_map<int,std::shared_ptr<Chunk>>> chunks;
    std::vector<std::shared_ptr<Chunk>> active_chunks;

    void load_chunk(ChunkedFieldLocation loc) {
        T container = chunk_create_callback(loc.i,loc.j);
        auto chunk = std::make_shared<Chunk>(container, loc);
        chunks[loc.i][loc.j] = chunk;
        active_chunks.push_back(chunk);
    }

    void unload_chunk(ChunkedFieldLocation loc) {
        chunks[loc.i][loc.j] = nullptr;
    }

    void cull_dead_chunks() {
        for(uint32_t i = 0; i < active_chunks.size(); i++) {
            if(!active_chunks[i]->keep_loaded_flag) {
                auto loc = active_chunks[i]->location;
                chunks[loc.i][loc.j] = nullptr;
                active_chunks.erase(active_chunks.begin()+i);
            }
        }
    }

    void world_pos_to_idx() {

    }

public:
    std::function<T(int,int)> chunk_create_callback;

    ChunkedField() {

    }

    void register_chunk_create_callback(std::function<T(int,int)> callback) {
        chunk_create_callback = callback;
    }

    void poll_chunk(ChunkedFieldLocation loc) {
        if(chunks[loc.i][loc.j] == nullptr) {
            load_chunk(loc);
        } else {
            update_chunk(loc);
        }
    }

    void update_chunk(ChunkedFieldLocation loc) {
        chunks[loc.i][loc.j]->keep_loaded_flag = true;
    }

    std::shared_ptr<Chunk> get_chunk(ChunkedFieldLocation loc) {
        return chunks[loc.i][loc.j];
    }

    T get_chunk_cont(ChunkedFieldLocation loc) {
        return chunks[loc.i][loc.j]->container;
    }

    void pre_update() {
        for(auto c : active_chunks) {
            c->keep_loaded_flag = false;
        }
    }

    void post_update() {
        cull_dead_chunks();
    }

};

#endif //HINA_SSI_PLUGIN_CHUNK_FIELD_H

#include "render_config.h"

render_config* render_config::INSTANCE = nullptr;

render_config* render_config::getInstance() {
    if(INSTANCE == nullptr) {
        INSTANCE = new render_config();
    }
    return INSTANCE;
}
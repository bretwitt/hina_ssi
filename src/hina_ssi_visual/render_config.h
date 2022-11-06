#ifndef HINA_SSI_PLUGIN_RENDER_CONFIG_H
#define HINA_SSI_PLUGIN_RENDER_CONFIG_H

class render_config {

private:
    static render_config* INSTANCE;

public:

    static render_config* getInstance();

    ~render_config() {
        delete INSTANCE;
    }

    enum SoilRenderMode {
        WIREFRAME, SOLID, SOLID_NO_SHADOWS
    };

    SoilRenderMode render_mode = WIREFRAME;
};




#endif //HINA_SSI_PLUGIN_RENDER_CONFIG_H

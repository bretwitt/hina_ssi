#ifndef HINA_SSI_SOIL_PHYS_PARAMS_H
#define HINA_SSI_SOIL_PHYS_PARAMS_H

struct SoilPhysicsParams {
    double k_phi = 814000.0f; //814000.0f;
    double k_e = 7.8e7;
    double k_c = 20680.0f;
    double c = 3500;
    double phi = 0.55;
    double mfr = 0.25;
};

#endif //HINA_SSI_SOIL_PHYS_PARAMS_H

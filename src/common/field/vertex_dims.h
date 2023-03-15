#ifndef HINA_SSI_PLUGIN_VERTEX_DIMS_H
#define HINA_SSI_PLUGIN_VERTEX_DIMS_H

/*
 *  Counts size of field as the number of vertices on each axis
 */
struct FieldVertexDimensions {
    uint32_t verts_x;
    uint32_t verts_y;
};

/*
 * Counts size of field as the number of meters of each axis
 */

struct FieldTrueDimensions {
    double true_x;
    double true_y;
};

#endif //HINA_SSI_PLUGIN_VERTEX_DIMS_H

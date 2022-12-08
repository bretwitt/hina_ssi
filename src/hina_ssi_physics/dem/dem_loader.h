#ifndef HINA_SSI_PLUGIN_DEM_LOADER_H
#define HINA_SSI_PLUGIN_DEM_LOADER_H

#include "dem.h"
#include <gdal/gdal_priv.h>
#include <gdal/cpl_conv.h>

using namespace gazebo;

namespace hina {
    class DEMLoader {

    public:
        static std::shared_ptr<DEM> load_dem_from_geotiff(const std::string& file) {
            std::vector<std::vector<double>> height_map;

            GDALDataset* dataset;
            GDALAllRegister();
            dataset = (GDALDataset*) GDALOpen(file.c_str(), GA_ReadOnly);
            if( dataset == nullptr ) {
                std::cerr << "Error opening TIFF as GDALDataset" << std::endl;
                return nullptr;
            }

            GDALRasterBand* heightband = dataset->GetRasterBand(1);

            float* heightScanBuf;
            int nXSize = heightband->GetXSize();
            int nYSize = heightband->GetYSize();

            double geoTransform[6];
            dataset->GetGeoTransform(geoTransform);
            double res = geoTransform[1];

            std::shared_ptr<DEM> dem = std::make_shared<DEM>(FieldVertexDimensions{ static_cast<double>(nXSize),
                                                                                    static_cast<double>(nYSize)},res);

            for(uint32_t y_line = 0; y_line < nYSize; y_line++) {
                heightScanBuf = (float*)CPLMalloc(sizeof(float)*nXSize);
                heightband->RasterIO(GF_Read, 0, y_line, nXSize, 1, heightScanBuf, nXSize, 1, GDT_Float32, 0, 0);
                for(uint32_t x = 0; x < nXSize; x++) {
                    double z = heightScanBuf[x];
                    dem->load_vertex(x+(y_line*nXSize), z);
                }
            }

            CPLFree(heightScanBuf);

            if(dataset->GetRasterCount() > 1) {
                GDALRasterBand* maskband = dataset->GetRasterBand(2);
                bool* maskScanBuf;

                for(uint32_t y_line = 0; y_line < nYSize; y_line++) {
                    maskScanBuf = (bool*)CPLMalloc(sizeof(float)*nXSize);
                    maskband->RasterIO(GF_Read, 0, y_line, nXSize, 1, maskScanBuf, nXSize, 1, GDT_Byte, 0, 0);
                    for(uint32_t x = 0; x < nXSize; x++) {
                        auto vtx = dem->field->get_vertex_at_index(x,y_line)->v;
                        vtx->isAir = maskScanBuf[x];
                    }
                }
                CPLFree(maskScanBuf);
            }

            GDALClose(dataset);

            return dem;
        }

        static std::shared_ptr<DEM> load_dem_from_csv(const std::string &file) {

            std::vector<std::vector<double>> height_map;

            std::ifstream f;
            f.open(file);

            if (!f.is_open()) {
                std::cerr << "Error: File Open Failed" << std::endl;
            }

            std::stringstream ss;
            std::string line;
            double w_x;
            double w_y;

            double res;
            double target_res;

            std::getline(f, line, ',');
            w_x = std::stod(line);              // true width
            std::getline(f, line, ',');
            w_y = std::stod(line);              // true width
            std::getline(f, line, ',');
            res = std::stod(line);
            std::getline(f, line, ',');
            target_res = std::stod(line);

            std::shared_ptr<DEM> dem = std::make_shared<DEM>(FieldTrueDimensions { w_x, w_y },res);

            int i = 0;

            while (std::getline(f, line, ',')) {
                double z = std::stod(line);
                dem->load_vertex(i, z);
                i++;
            }

            f.close();

            dem->upsample(target_res);

            return dem;
        }
    };
}
#endif //HINA_SSI_PLUGIN_DEM_LOADER_H

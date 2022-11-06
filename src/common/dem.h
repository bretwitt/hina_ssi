#include "soil.h"
#include <cmath>
#include <omp.h>

using namespace gazebo;


std::vector<std::vector<float>> Soil::loadDemHeights(int newsize, int originalSize, std::string &filename) {
    // change to work with non squared later
    
    float scale = float(originalSize) / float(newsize);
    const int osize = originalSize;
    std::ifstream f;
    f.open(filename);

    if (!f.is_open()) {    /* validate file open for reading */
        std::cerr << "error: file open failed '" << "'.\n";
        //return 1;
    }

    std::vector<std::vector<float>> data(originalSize);
    for (int row = 0; row < originalSize; ++row)
    {
        std::string line;
        std::getline(f, line);
        std::stringstream s(line);
        data[row] = std::vector<float>(originalSize);

        for (int col = 0; col < originalSize; ++col)
        {
            std::string val;
            std::getline(s, val, ',');
            data[row][col] = std::stof(val);
        }
    }
    std::vector<std::vector<float>> demHeight(newsize);
    scale = float(originalSize) / float(newsize);
    for (int i = 0; i < newsize; ++i)
    {
        demHeight[i] = std::vector<float>(newsize);

        for (int j = 0; j < newsize; ++j)
        {
            demHeight[i][j] = data[int(i * scale)][int(j * scale)];
        }
    }
    return demHeight;
}



std::vector<std::vector<int>> Soil::loadDemLabels(int newsize, int originalSize, std::string& filename) {
    // change to work with non squared later

    float scale = float(originalSize) / float(newsize);
    const int osize = originalSize;
    std::ifstream f;
    f.open(filename);

    if (!f.is_open()) {    /* validate file open for reading */
        std::cerr << "error: file open failed '" << "'.\n";
        //return 1;
    }

    std::vector<std::vector<int>> data(originalSize);

    for (int row = 0; row < originalSize; ++row)
    {
        std::string line;
        std::getline(f, line);
        std::stringstream s(line);
        data[row] = std::vector<int>(originalSize);

        for (int col = 0; col < originalSize; ++col)
        {
            std::string val;
            std::getline(s, val, ',');
            data[row][col] = std::stoi(val);
        }
    }

    std::vector<std::vector<int>> demHeight(newsize);
    scale = float(originalSize) / float(newsize);
    for (int i = 0; i < newsize; ++i)
    {
        demHeight[i] = std::vector<int>(newsize);

        for (int j = 0; j < newsize; ++j)
        {
            demHeight[i][j] = data[int(i * scale)][int(j * scale)];
        }
    }
    return demHeight;
}



void Soil::generate_DEM_vertices() {
    _data->x_offset = -(double) (_data->x_width - 1) / 2;
    _data->y_offset = -(double) (_data->y_width - 1) / 2;

    // need better way to set file path
    std::string fname = "D:/Colab stuff/savedModels/testHeight.csv";
    std::string Lname = "D:/Colab stuff/savedModels/testLabels.csv";

    std::vector<std::vector<float>> heightMap = loadDem(propSize, n, fname);
    std::vector<std::vector<int>> l = loadDemLabels(propSize, n, Lname);

    struct TerrainDEM d1(_data->x_width, _data->y_width, _data->scale, _data->x_offset, _data->y_offset, heightMap, l);
}



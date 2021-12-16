#ifndef __OCCUPANCY_GRID_H__
#define __OCCUPANCY_GRID_H__

#include <vector>
#include "SensorMappingDefinitions.h"

class OccupancyGrid
{
public:
    OccupancyGrid(int rows, int cols, double resolution);
    ~OccupancyGrid();
    static void print(const OccupancyGrid &grid);

    char** grid;
    int cols;
    int rows;
    double resolution;
    std::vector<LaserRay> laserRays;
    std::vector<Position> intermediatePoints;
};

#endif
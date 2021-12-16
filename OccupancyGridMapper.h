#ifndef __OCCUPANCY_GRID_MAPPER_H__
#define __OCCUPANCY_GRID_MAPPER_H__

#include "SensorMappingDefinitions.h"
#include "OccupancyGrid.h"

class OccupancyGridMapper
{
public:
    OccupancyGridMapper(LasersData lasersData, Pose laser1Pose, Pose laser2Pose, Position areaSize, double resolution);
    ~OccupancyGridMapper();
    
    OccupancyGrid* getOccupancyGrid() { return m_occupancyGrid; };
    OccupancyGrid* computeOccupancyGrid();
    void printLaserData(LaserData data);

private:
    LaserData m_laser1Data;
    LaserData m_laser2Data;
    Pose m_laser1Pose;
    Pose m_laser2Pose;

    OccupancyGrid* m_occupancyGrid;

    void computeOccupancySingleLaser(LaserData data, Pose pose, bool dontBypassExistingProbabilities);
    void computeOccupancySingleRay(Position source, Position target, double distance, double cosAngle, double sinAngle, bool dontBypassExistingProbabilities);
    bool rayInterceptsSquare(Position squareVertices[4], double a, double b, double c, double resolution);
};

#endif
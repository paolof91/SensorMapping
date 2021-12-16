#ifndef __SENSOR_MAPPING_DEFINITIONS_H__
#define __SENSOR_MAPPING_DEFINITIONS_H__

#include <map>
#include <tuple>

#define M_PI 3.14159265358979323846  /* pi */
#define EPS_PRECISION 1e-4 /* 0.1mm = 100um */

struct Position
{
    double x;
    double y;
};

struct Dimension
{
    double width;
    double height;
};


struct Pose
{
    Position position;
    double angle; // rads
};

using LaserData = std::map<double,double>; // rads, m
using LasersData = std::tuple<LaserData,LaserData>;
using LaserRay = std::tuple<Position,Position>;

#endif
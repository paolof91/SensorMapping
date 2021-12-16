/**
 * @file SensorMapping.cpp
 * @brief Main function of the SensorMapping program
 * @author Paolo Forni
 */
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <tuple>
#include "math.h"

#include "OccupancyGridMapper.h"
#include "SensorMappingDefinitions.h"
#include "RectangleDetector.h"

using namespace cv;

// main function headers
LasersData readLaserInput();

/**
 * @function main
 * @brief Main function
 */
int main( void )
{
	Pose laser1Pose = {0.0, 0.0, M_PI/2};
	Pose laser2Pose = {15.0, 0.0, M_PI};
	Position areaSize = {15.0, 10.0};
	double resolution = 0.5;

	LasersData lasersData = readLaserInput();

	OccupancyGridMapper mapper(lasersData, laser1Pose, laser2Pose, areaSize, resolution);
	OccupancyGrid* grid = mapper.computeOccupancyGrid();

	RectangleDetector rectangleDetector(grid);
	rectangleDetector.generateOccupancyGridImage();
	rectangleDetector.detectRectangle();
	rectangleDetector.saveData();

	return(0);
}

/**
 * @function readLaserInput()
 * @brief Reads from file input_L1_L2.txt and returns two lists of angle-distance laser data
 */
LasersData readLaserInput()
{
	LaserData inputL1;
	LaserData inputL2;
	std::ifstream infile("input_L1_L2.txt");
	std::string line;
	while (std::getline(infile, line))
	{
		std::istringstream iss(line);
		double a, b, c;
		if (!(iss >> a >> b >> c))
		{
			break;  // error
		}
		inputL1.insert({M_PI*a/180.0,b});
		inputL2.insert({M_PI*a/180.0,c});
	}
	return std::make_tuple(inputL1,inputL2);
}

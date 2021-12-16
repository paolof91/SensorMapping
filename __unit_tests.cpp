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

#define EPS_UNIT_TEST 0.02

using namespace cv;

// main function headers
LasersData test1Input();
bool test1_borderConditions();
bool test2_detectStraightRectangle();
bool test3_detectRotatedRectangle();

/**
 * @function main
 * @brief Main function
 */
int main( void )
{
    bool res = false;

    // Test 1: compute occupancy and border conditions
    std::cout << "_______________________________\nTEST 1: Compute occupancy and BORDER CONDITIONS\nSome laser points are exactly on the border between grid cells, or even corners\n" << std::endl;
    res = test1_borderConditions();
    if (res)
        std::cout << "---> test 1 returned SUCCESSFULLY\n" << std::endl;
    else
        std::cout << "---> test 1 FAILED \n" << std::endl;
    std::cout << "_______________________________" << std::endl;

    // Test 2: detect straight rectangle given occupancy grid
    std::cout << "_______________________________\nTEST 2: Straight rectangle detection \n" << std::endl;
    res = test2_detectStraightRectangle();
    if (res)
        std::cout << "---> test 2 returned SUCCESSFULLY\n" << std::endl;
    else
        std::cout << "---> test 2 FAILED \n" << std::endl;
    std::cout << "_______________________________" << std::endl;

    // Test 3: detect rotated rectangle given occupancy grid
    std::cout << "_______________________________\nTEST 3: Rotated rectangle detection \n" << std::endl;
    res = test3_detectRotatedRectangle();
    if (res)
        std::cout << "---> test 3 returned SUCCESSFULLY\n" << std::endl;
    else
        std::cout << "---> test 3 FAILED \n" << std::endl;
    std::cout << "_______________________________" << std::endl;

	return(0);
}

/**
 * @function test1_borderConditions
 * @brief Test 1: compute occupancy and border conditions
 */
bool test1_borderConditions()
{
    // test input
	Pose laser1Pose = {0.0, 0.0, M_PI/2};
	Pose laser2Pose = {15.0, 0.0, M_PI};
	Position areaSize = {15.0, 10.0};
	double resolution = 5;
	LasersData lasersData = test1Input();

    // test output
    char data[2][3] = { {0,0,0}, {1,1,0} };

    // computation
	OccupancyGridMapper mapper(lasersData, laser1Pose, laser2Pose, areaSize, resolution);
	OccupancyGrid* grid = mapper.computeOccupancyGrid();

    // checks
    bool res = 1;

    size_t i,j;
    for(i=0;i<grid->rows;i++)
    {
        for(j=0;j<grid->cols; j++)
        {
            if (grid->grid[i][j] != data[i][j] )
            {
                std::cout << "(i,j) = (" << i << "," << j << ")" << std::endl;
                std::cout << grid->grid[i][j];
                res = 0;
                break;
            }
        }
        if (res==0)
            break;
    }
    return res;
}

/**
 * @function test2_detectStraightRectangle
 * @brief Test 2: detect straight rectangle given occupancy grid
 */
bool test2_detectStraightRectangle()
{
    // test input
    char data[10][15] = {   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 1, 2, 2, 2, 2, 1, 0, 0, 0, 0, 0},
                            {0, 0, 0, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 2, 2, 2, 2, 2, 0, 2, 0, 2, 0, 0}};
    OccupancyGrid grid(10,15,1.0);
    size_t i,j;
	for(i=0;i<grid.rows;i++)
		for(j=0;j<grid.cols;j++)
            grid.grid[i][j]=data[i][j];

    // computation
    RectangleDetector rectangleDetector(&grid);
	rectangleDetector.generateOccupancyGridImage();
	rectangleDetector.detectRectangle();

    // checks
    bool res = 1;
    if ( fabs(rectangleDetector.rectBottomLeftCorner.x-4.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rectBottomLeftCorner.y-5.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rectDimension.width-6.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rectDimension.height-2.0)>EPS_UNIT_TEST )
         res = 0;
    if ( fabs(rectangleDetector.rotatedRectCenter.x-7.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rotatedRectCenter.y-6.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rotatedRectDimension.width-2.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rotatedRectDimension.height-6.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rotatedRectAngleDeg-90)>5.0*EPS_UNIT_TEST )
         res = 0;

    return res;
}

/**
 * @function test3_borderConditions
 * @brief Test 3: detect rotated rectangle given occupancy grid
 */
bool test3_detectRotatedRectangle()
{
    // test input
    char data[10][15] = {   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 2, 2, 2, 2, 1, 0, 0, 0, 0, 0},
                            {0, 0, 0, 2, 1, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 2, 2, 2, 2, 2, 0, 2, 0, 2, 0, 0}};
    OccupancyGrid grid(10,15,1.0);
    size_t i,j;
	for(i=0;i<grid.rows;i++)
		for(j=0;j<grid.cols;j++)
            grid.grid[i][j]=data[i][j];

    // computation
    RectangleDetector rectangleDetector(&grid);
	rectangleDetector.generateOccupancyGridImage();
	rectangleDetector.detectRectangle();

    // checks
    bool res = 1;
    if ( fabs(rectangleDetector.rectBottomLeftCorner.x-4.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rectBottomLeftCorner.y-5.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rectDimension.width-6.0)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rectDimension.height-3.0)>EPS_UNIT_TEST )
         res = 0;
    if ( fabs(rectangleDetector.rotatedRectCenter.x-6.85)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rotatedRectCenter.y-6.22)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rotatedRectDimension.width-6.26)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rotatedRectDimension.height-2.73)>EPS_UNIT_TEST ||
         fabs(rectangleDetector.rotatedRectAngleDeg-11.30)>5.0*EPS_UNIT_TEST )
         res = 0;

    return res;
}

/**
 * @function test1Input()
 * @brief Creates the input for test 1
 */
LasersData test1Input()
{
	LaserData inputL1;
	LaserData inputL2;

    // one reading is exactly on the corner
    inputL1.insert({0, 10});
    inputL1.insert({M_PI/4.0, sqrt(2)*5});
    inputL1.insert({M_PI/2, 15});

    // one reading is exactly on the corner
    inputL2.insert({0, 15});
    inputL2.insert({0.46364760, 11.1803398});
    inputL2.insert({M_PI/2, 10});

	return std::make_tuple(inputL1,inputL2);
}

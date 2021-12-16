#include <iostream>
#include "OccupancyGrid.h"

OccupancyGrid::OccupancyGrid(int rows, int cols, double resolution)
{
    this->rows = rows;
    this->cols = cols;
    this->resolution = resolution;

    grid = new char*[rows];
    int i,j;
    for(i=0; i<rows; i++)
    {
        grid[i] = new char[cols];
        for(j=0; j<cols; j++)
            grid[i][j] = 2;
    }
}

OccupancyGrid::~OccupancyGrid()
{
    int i;
    for(i=0; i<rows; i++)
        delete [] grid[i];
    delete [] grid;
}

void OccupancyGrid::print(const OccupancyGrid &inputGrid)
{
    size_t i,j;
	for(i=0;i<inputGrid.rows;i++)
	{
		std::cout<< "{";
		for(j=0;j<inputGrid.cols;j++)
		{
			std::cout<< (int)inputGrid.grid[i][j] << ",";

		}
        std::cout<< inputGrid.grid[i][j] << "},\n";
		
	}
    std::cout << "}\n";
}

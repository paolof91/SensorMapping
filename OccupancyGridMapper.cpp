#include <iostream>
#include <tuple>
#include <math.h>

#include "OccupancyGridMapper.h"

OccupancyGridMapper::OccupancyGridMapper(LasersData lasersData, Pose laser1Pose, Pose laser2Pose, Position areaSize, double resolution)
{
    std::tie(m_laser1Data,m_laser2Data) = lasersData;

    m_laser1Pose = laser1Pose;
    m_laser2Pose = laser2Pose;

    m_occupancyGrid = new OccupancyGrid(ceil(areaSize.y/resolution),
                                    ceil(areaSize.x/resolution),
                                    resolution);
}

OccupancyGridMapper::~OccupancyGridMapper()
{
    delete m_occupancyGrid;
}

OccupancyGrid* OccupancyGridMapper::computeOccupancyGrid()
{
    computeOccupancySingleLaser(m_laser1Data, m_laser1Pose, false);
    computeOccupancySingleLaser(m_laser2Data, m_laser2Pose, true);

    return m_occupancyGrid;
}

/**
 * @function computeOccupancySingleLaser(LaserData data, Pose pose, double dontBypassExistingProbabilities)
 * @brief generates source and target points for each measurements and computes occupancy of each ray
 */
void OccupancyGridMapper::computeOccupancySingleLaser(LaserData data, Pose pose, bool dontBypassExistingProbabilities)
{
    Position source = pose.position;
    Position target;
    double cosAngle, sinAngle, distance;
    for (const auto& measurement : data)
    {
        cosAngle = cos(pose.angle-measurement.first);
        sinAngle = sin(pose.angle-measurement.first);
        distance = measurement.second;
        target.x = source.x+distance*cosAngle;
        target.y = source.y+distance*sinAngle;

        computeOccupancySingleRay(source,target,distance,cosAngle,sinAngle,dontBypassExistingProbabilities);
        m_occupancyGrid->laserRays.push_back(std::make_tuple(source, target));
    }
}

/**
 * @function computeOccupancySingleRay(Position source, Position target, double dontBypassExistingProbabilities)
 * @brief given source and target of the laser ray, computes the occupancy on the grid
 */
void OccupancyGridMapper::computeOccupancySingleRay(Position source, Position target, double distance, double cosAngle, double sinAngle, bool dontBypassExistingProbabilities)
{
    double resolution =  m_occupancyGrid->resolution;
    double borderThreshold = resolution/20.0;
    double currentDistance, stepDistance;
    int row, minRow, maxRow, deltaMinRow, deltaMaxRow;
    int col, minCol, maxCol, deltaMinCol, deltaMaxCol;
    stepDistance = resolution/1.0;
    int N = floor(distance/stepDistance);
    bool isTarget;
    Position currentPosition;
    Position squareVertices[4];

    // defining the 2x2 neighborhood of grid cells where to check for the ray to pass. This is a robustness measure
    if ( cosAngle>0 )
    {
        deltaMinCol = -1; deltaMaxCol = 0;
    }
    else
    {
        deltaMinCol = 0; deltaMaxCol = 1;
    }
    if ( sinAngle>0 )
    {
        deltaMinRow = -1; deltaMaxRow = 0;
    }
    else
    {
        deltaMinRow = 0; deltaMaxRow = 1;
    }

    int w,i,j;

    // takes steps along the ray going from laser source to target
    for(int w=0; w<N+2;w++)
    {
        // By our own definition, w==N means the current position is the target
        if (w==N+1)
        {
            currentPosition.x = target.x;
            currentPosition.y = target.y;
            isTarget = true;
        }
        else
        {
            currentDistance = w*stepDistance;
            currentPosition.x = source.x+currentDistance*cosAngle;
            currentPosition.y = source.y+currentDistance*sinAngle;
            m_occupancyGrid->intermediatePoints.push_back(currentPosition);
            isTarget = false;
        }
        row = fmax(0,fmin(floor(currentPosition.y/resolution), m_occupancyGrid->rows-1));
        col = fmax(0,fmin(floor(currentPosition.x/resolution), m_occupancyGrid->cols-1));
        /*row = floor(currentPosition.y/resolution);
        col = floor(currentPosition.x/resolution);
        */

        // defining the 2x2 neighborhood of grid cells where to check for the ray to pass. This is a robustness measure
        minCol = fmin(fmax(0,col+deltaMinCol), m_occupancyGrid->cols-1);
        maxCol = fmin(col+deltaMaxCol, m_occupancyGrid->cols-1);
        minRow = fmin(fmax(0,row+deltaMinRow), m_occupancyGrid->rows-1);
        maxRow = fmin(row+deltaMaxRow, m_occupancyGrid->rows-1);

        // check whether ray passes through the squares of a 2 x 2 grid before and around current point. This is for robustness.
        for( i = minRow; i<=maxRow; i++)
        {
            for( j = minCol; j<=maxCol; j++)
            {
                squareVertices[0].x = j*resolution; squareVertices[0].y = i*resolution;
                squareVertices[1].x = (j+1)*resolution; squareVertices[1].y = i*resolution;
                squareVertices[2].x = (j+1)*resolution; squareVertices[2].y = (i+1)*resolution;
                squareVertices[3].x = j*resolution; squareVertices[3].y = (i+1)*resolution;

                // if ray passes through the square, then we know occupance=0
                if ( rayInterceptsSquare(squareVertices, sinAngle, -cosAngle, -sinAngle*source.x+cosAngle*source.y, resolution) )
                    if ( m_occupancyGrid->grid[i][j] != 1) // if this cell is already occupied, don't overwrite it
                        m_occupancyGrid->grid[i][j] = 0;
            }
        }

        // if is target, just set the current cell to occupance = 1;
        if (isTarget)
        {
            // border conditions on x
            if ( fabs(target.x-resolution*round(target.x/resolution)) < borderThreshold )
            {
                col =  round(target.x/resolution);
                if (cosAngle<0)
                    col--;
            }

            // border conditions on y
            if ( fabs(target.y-resolution*round(target.y/resolution)) < borderThreshold )
            {
                row =  round(target.y/resolution);
                if (sinAngle<0)
                    row--;
            }

            if (row<m_occupancyGrid->rows && col<m_occupancyGrid->cols && row>=0 && col>=0)
                m_occupancyGrid->grid[row][col] = 1;
        }       
    }
}

/**
 * @function rayInterceptsSquare(Position squareVertices[4], double a, double b, double c)
 * @brief given the implicit equation of the ray (a x + b y + c = 0) and the 4 vertices of a square and the resolution (square side),
 * returns wether or not the ray passes through the current square
 */
bool OccupancyGridMapper::rayInterceptsSquare(Position squareVertices[4], double a, double b, double c, double resolution)
{
    bool signChange = false;
    signChange = a*squareVertices[0].x+b*squareVertices[0].y+c+EPS_PRECISION>0;
    
    for(int i=1; i<4; i++)
    {
        if (!((a*squareVertices[i].x+b*squareVertices[i].y+c+EPS_PRECISION>0) && signChange))
            return true;
    }
    return false;
}

void OccupancyGridMapper::printLaserData(LaserData data)
{
    for (const auto& laserPoint : data) {
        std::cout << laserPoint.first  << " " << laserPoint.second << "\n";
    }
    std::cout << "\n";
}

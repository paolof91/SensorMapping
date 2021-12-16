#ifndef __RECTANGLE_DETECTOR_H__
#define __RECTANGLE_DETECTOR_H__

#include <opencv2/core.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include "SensorMappingDefinitions.h"
#include "OccupancyGrid.h"

#define windowWidth 1500
#define windowHeight 1000
#define magnification 100 // 100 pixels mean 1m

using namespace cv;
using namespace std;

class RectangleDetector
{
public:
    RectangleDetector(OccupancyGrid* grid);
    
    void generateOccupancyGridImage();
    void detectRectangle();
    void saveData();

    Position rectBottomLeftCorner;
    Dimension rectDimension;
    Position rotatedRectCenter;
    Dimension rotatedRectDimension;
    double rotatedRectAngleDeg;

private:
    void MyLine( Mat img, Point start, Point end, Scalar color);

    OccupancyGrid* m_occupancyGrid;
    Mat m_gridMat;
    Mat m_fullMat;
    RotatedRect m_rotatedRect;
    Rect m_rect;

    bool m_hasGridMatBeenGenerated = false;
    bool m_hasRectanglesBeenDetected = false;
};

#endif
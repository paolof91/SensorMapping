#include <iostream>
#include <fstream>
#include <tuple>
#include "math.h"

#include "RectangleDetector.h"

RectangleDetector::RectangleDetector(OccupancyGrid* grid) : m_gridMat(Mat::zeros( windowHeight, windowWidth, CV_8UC3 )),
                                                            m_fullMat(Mat::zeros( windowHeight, windowWidth, CV_8UC3 )) 
{
    m_occupancyGrid = grid;
}


/**
 * @function generateOccupancyGridImage()
 * @brief Creates the grid image
 */
void RectangleDetector::generateOccupancyGridImage()
{
	// setting semaphores
	m_hasGridMatBeenGenerated = false;
	m_hasRectanglesBeenDetected = false;
	std::cout << "Generating Occupancy Grid Image..." << std::endl;

    // initializing window name and Mat
	char windowName[] = "Occupancy Grid";

	// drawing as many rectangles as occupancy grid cells
	double resolution = m_occupancyGrid->resolution;
	int i,j;
	int x1,x2,y1,y2;
	for(i=0;i<m_occupancyGrid->rows;i++)
	{
		for(j=m_occupancyGrid->cols-1;j>=0;j--)
		{
			x1 = floor(magnification*resolution*j);
			x2 = floor(magnification*resolution*(j+1));
			y1 = windowHeight-floor(magnification*resolution*(i+1));
			y2 = windowHeight-floor(magnification*resolution*i);

			x1 = fmax(fmin(windowWidth,x1),0);
			x2 = fmax(fmin(windowWidth,x2),0);
			y1 = fmax(fmin(windowHeight,y1),0);
			y2 = fmax(fmin(windowHeight,y2),0);

			switch(m_occupancyGrid->grid[i][j])
			{
				case 1:
					rectangle( m_gridMat,
								Point( x1, y1 ),
								Point( x2, y2),
								Scalar(0,0,0),
								FILLED,
								LINE_8 );
					break;
				case 0:
					rectangle( m_gridMat,
								Point( x1, y1 ),
								Point( x2, y2),
								Scalar(255,255,255),
								FILLED,
								LINE_8 );
					break;
				default:
					rectangle( m_gridMat,
								Point( x1, y1 ),
								Point( x2, y2),
								Scalar(128,128,128),
								FILLED,
								LINE_8 );
					break;
			}
		}
	}

    // Display the grid
	imshow( windowName, m_gridMat );
	moveWindow( windowName, 200, 80 );

	// setting semaphores
    m_hasGridMatBeenGenerated = true;
	std::cout << "Occupancy Grid Image successfully generated." << std::endl;
}

/**
 * @function detectRectangle()
 * @brief Creates the debug grid and detects the rectangle
 */
void RectangleDetector::detectRectangle()
{
	// checking/setting semaphores
	// setting semaphores
	m_hasRectanglesBeenDetected = false;
    if (!m_hasGridMatBeenGenerated)
	{
		std::cout << "Detect rectangles returned with error:\nOccupancy Grid Image must be generated first." << std::endl;
		return;
	}
	std::cout << "Detecting straight and rotated rectangles..." << std::endl;

    // initializing window name and Mat
	char windowName[] = "Occupancy Grid - Debug";

    // copy the grid Mat onto a new Mat
    m_fullMat = m_gridMat.clone();
    // convert to grayscale (you could load as grayscale instead)
    Mat grayMat;
    cvtColor(m_fullMat, grayMat, COLOR_BGR2GRAY);

    // compute mask (you could use a simple threshold if the image is always as good as the one you provided)
    Mat maskMat;
    threshold(grayMat, maskMat, 80, 255, THRESH_BINARY_INV );

	// Find contours
	std::vector<std::vector<Point> > contours;
	findContours(maskMat.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	// Sum contours
	std::vector<Point> contoursSum;
	for (std::vector<Point> & v : contours)
		for (Point& p : v)
			contoursSum.push_back(p);

	// generate convex hull
	std::vector<Point> hull( contoursSum.size() );
    for( size_t i = 0; i < contoursSum.size(); i++ )
    {
        convexHull( contoursSum, hull );
    }

	// draw convex hull
	Point p_prev = hull.at(0);
	for (Point& p : hull)
	{
		MyLine( m_fullMat, p_prev, p, Scalar( 0, 0, 255 ));
		p_prev = p;
	}

	// find minimum bounding rotated rect
	m_rotatedRect = minAreaRect(hull);

	// draw rotated rect 
	Point2f vertices[4];
	m_rotatedRect.points(vertices);
	for (  size_t i = 0; i < 4; i++ )
		MyLine( m_fullMat, vertices[i], vertices[(i+1)%4] , Scalar( 0, 255, 0 ));

	// find minimum bounding straight rect
	m_rect = boundingRect(hull);

	// draw rotated rect 
	rectangle( m_fullMat, m_rect.br(), m_rect.tl(), Scalar(0,255,255), 2);

	// save key informations about the straight rectangle
	rectBottomLeftCorner.x = (double)m_rect.x/magnification;
	rectBottomLeftCorner.y = (double)(windowHeight-m_rect.height-m_rect.y)/magnification;
	rectDimension.width = (double)m_rect.width/magnification;
	rectDimension.height = (double)m_rect.height/magnification;

	// save key informations about the rotated rectangle
	rotatedRectCenter.x = (double)m_rotatedRect.center.x/magnification;
	rotatedRectCenter.y = (double)(windowHeight-m_rotatedRect.center.y)/magnification;
	rotatedRectDimension.width = (double)m_rotatedRect.size.width/magnification;
	rotatedRectDimension.height = (double)m_rotatedRect.size.height/magnification;
	rotatedRectAngleDeg = (double)m_rotatedRect.angle;

    // drawing the laser rays
	Position source,target;
    int x1,x2,y1,y2;
	for (const auto& laserRay : m_occupancyGrid->laserRays)
	{
	    std::tie(source,target) = laserRay;
		x1 = round(source.x*magnification);
		x2 = round(target.x*magnification);
		y1 = round(source.y*magnification);
		y2 = round(target.y*magnification);
		x1 = fmax(fmin(windowWidth,x1),0);
		x2 = fmax(fmin(windowWidth,x2),0);
		y1 = windowHeight-fmax(fmin(windowHeight,y1),0);
		y2 = windowHeight-fmax(fmin(windowHeight,y2),0);		
		MyLine( m_fullMat, Point( x1, y1 ), Point( x2, y2 ),Scalar( 255, 0, 0 ) );
	}

	// drawing the intermediatePoints
	for (const auto& intermediatePoint : m_occupancyGrid->intermediatePoints)
	{
		x1 = round(intermediatePoint.x*magnification);
		y1 = round(intermediatePoint.y*magnification);
		x1 = fmax(fmin(windowWidth,x1),0);
		y1 = windowHeight-fmax(fmin(windowHeight,y1),0);
		ellipse( m_fullMat,
			Point( x1, y1 ),
			Size( 3,3 ),
			0,
			0,
			360,
			Scalar( 0, 0, 255 ),
			2,
			8 );
	}

    // Display the grid
	imshow( windowName, m_fullMat );
	moveWindow( windowName, 200, 80 );

	// setting semaphores
    m_hasRectanglesBeenDetected = true;
	std::cout << "Straight and rotated rectangles successfully generated." << std::endl;
}

/**
 * @function saveData()
 * @brief Saves the grid into an image an detected rectangle dimensions into a text file
 */
void RectangleDetector::saveData()
{
    if(!m_hasGridMatBeenGenerated && !m_hasRectanglesBeenDetected)
	{
		std::cout << "Saving data return with error:\nRectangles must be detected first." << std::endl;
        return;
	}
	std::cout << "Saving data..." << std::endl;

	cv::imwrite("./output/occupancyGridMap.jpg", m_gridMat); 
	cv::imwrite("./output/occupancyGridMap__debug.jpg", m_fullMat); 

	std::ofstream myfile;
	myfile.open ("./output/rectangles.txt");
	myfile << "\nStraight rectangle:\n\tbottom left corner (x,y)=(";
	myfile << rectBottomLeftCorner.x << ",";
	myfile << rectBottomLeftCorner.y << ")\n\tsize (w,h)=(";
	myfile << rectDimension.width << ",";
	myfile << rectDimension.height << ")\n" << std::endl;

	myfile << "\nRotated rectangle:\n\tcentre (x,y)=(";
	myfile << rotatedRectCenter.x << ",";
	myfile << rotatedRectCenter.y << ")\n\tsize (w,h)=(";
	myfile << rotatedRectDimension.width << ",";
	myfile << rotatedRectDimension.height << ")\n\torientation [deg] angle=";
	myfile << rotatedRectAngleDeg << "\n" << std::endl;
  	myfile.close();

	std::cout << "\nData correctly saved in ./output/" << std::endl;
	std::cout << "See directory ./output/ for IMAGES and TEXT files." << std::endl;

}

/**
 * @function MyLine( Mat img, Point start, Point end )
 * @brief Draws a line
 */
void RectangleDetector::MyLine( Mat img, Point start, Point end, Scalar color)
{
  int thickness = 2;
  int lineType = LINE_8;

  line( img,
    start,
    end,
    color,
    thickness,
    lineType );
}
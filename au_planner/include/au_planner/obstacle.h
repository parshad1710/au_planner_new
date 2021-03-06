#include <math.h>


#ifndef OBSTACLE_H
#define OBSTACLE_H
struct Point
{
	float x;
	float y;
};

struct Obstacles //Make a typedef
{
	Point p[4];
};

class Obstacle
{
	public:
	
	bool onSegment(Point,Point,Point);
	int orientation(Point,Point,Point);
	bool doIntersect(Point,Point,Point,Point);
	bool isInside(Obstacles,int ,Point);
		
};

#endif

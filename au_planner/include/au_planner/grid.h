#include <math.h>
#include <au_planner/obstacle.h>
#include <vector>
#include <iostream>
#ifndef GRID_H
#define GRID_H

class Grid : public Obstacle
{
	float resolution;
	Point lower;
	Point upper;
	Point num_cells;
	public:
		std::vector<Obstacles> ob;
		Grid(float,Point,Point);
		~Grid(){};
		Point NodeIDtoGridCoord(float);
		Point NodeIDtoConfiguration(float);
		float GridCoordtoNodeID(Point);
		float ConfigurationtoNodeID(Point);
		Point GridCoordtoConfiguration(Point);
		Point ConfigurationtoGridCoord(Point);
		void AddObstacles(Obstacles);
		Point GetNumCells();
};

#endif

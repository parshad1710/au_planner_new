#include <au_planner/grid.h>

Grid::Grid(float res,Point l_l,Point u_l): resolution(res),lower(l_l),upper(u_l)
{
	num_cells.x = ceil((upper.x - lower.x)/resolution);
	num_cells.y = ceil((upper.y - lower.y)/resolution);
}
		
Point Grid::GetNumCells()
{
	return num_cells;
}

void Grid::AddObstacles(Obstacles obs)
{
	ob.push_back(obs);
}

Point Grid::NodeIDtoGridCoord(float nid)
{
	Point coord;
	coord.x = ((int)nid)%(int)num_cells.x;
	coord.y = (int)((nid)/num_cells.y);
  return coord;
}

Point Grid::GridCoordtoConfiguration(Point coord)
{
	Point config;
	config.x = lower.x + resolution*coord.x + resolution/2;
	config.y = lower.y + resolution*coord.y + resolution/2;
	return config;
}
	
Point Grid::NodeIDtoConfiguration(float nid)
{
	Point coord = NodeIDtoGridCoord(nid);
	Point config = GridCoordtoConfiguration(coord);
	return config;
}

Point Grid::ConfigurationtoGridCoord(Point config)
{
	Point coord;
	coord.x = floor((config.x - lower.x)/resolution);
	coord.y = floor((config.y - lower.y)/resolution);
	return coord;
}

float Grid::GridCoordtoNodeID(Point coord)
{
	float nid;
	nid = (coord.y)*num_cells.y + coord.x;
	return nid;
}

float Grid::ConfigurationtoNodeID(Point config)
{
	Point coord = ConfigurationtoGridCoord(config);
	float nid = GridCoordtoNodeID(coord);
	return nid;	
}

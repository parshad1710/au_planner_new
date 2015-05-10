#include <au_planner/grid.h>
#include <vector>
#include <map>
#include <set>

#ifndef PLANNER_H
#define PLANNER_H

class Planner:public Grid
{
	float resolution;
	Point start;
	Point goal;
	Point upper;
	Point lower;

	/*std::map<int,float> f_cost;
	std::map<int,float> g_cost;
	std::set<int> open_set;
	std::set<int> closed_set;
	std::map<int,int> nodes;*/
	std::set<float>	closed_set,open_set;
	std::map<float,float> node_map,g_cost,f_cost;
	
	public:
		Planner(float,Point,Point,Point,Point);
		~Planner(){};
		
		//Astar Planner
		void AstarPlan(std::vector<Point> &);
		void GetSuccessors(int nid,std::vector<float> &);
		float ComputeHeuristicCost(int,int);	
		float ComputeDistance(int,int);

		//RRT
		void GetRandom(Point &);
		

		//Common
		bool InCollision(Point);
		bool InBoundary(Point);
		void ShortenPlan(std::vector<Point> &);
		void LocalPlan(std::vector<Point> &);
		void SendPlan(std::vector<Point> &,std::vector< std::vector<Point> > &);
};

#endif

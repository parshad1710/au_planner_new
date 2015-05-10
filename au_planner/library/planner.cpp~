#include <au_planner/planner.h>
#include <complex>

Planner::Planner(float res,Point l_l,Point u_l,Point S,Point G): resolution(res),lower(l_l),upper(u_l),start(S),goal(G),Grid(res,l_l,u_l)
{
}

bool Planner::InCollision(Point p)
{
	int collision = 0;
	std::cout<<ob.size()<<std::endl;
	for(int i = 0 ; i<ob.size(); i++)
	{
		std::cout<<(ob.at(i)).p[0].x<<(ob.at(i)).p[0].y<<(ob.at(i)).p[1].x<<(ob.at(i)).p[1].y<<(ob.at(i)).p[2].x<<(ob.at(i)).p[2].y<<(ob.at(i)).p[3].x<<(ob.at(i)).p[3].y;
		std::cout<<"Collision Check\n";
		if(isInside(ob.at(i),4,p)) {collision++;}
	}
	if(collision!=0){
	std::cout<<"In Collision\n";
	return true;
	}
	else return false;
}

bool Planner::InBoundary(Point p)
{
	if(p.x>lower.x && p.x<upper.x && p.y>lower.y && p.y<upper.y)
	{
		return true;
	}
	return false;
}

float Planner::ComputeHeuristicCost(int S,int G)
{
	float cost = 0;
	Point s_config = NodeIDtoConfiguration((float)S);
	Point g_config = NodeIDtoConfiguration((float)G);
	cost = fabs(s_config.x-g_config.x)+fabs(s_config.y-g_config.y);
	return cost;	
}

float Planner::ComputeDistance(int S,int G)
{
	float dist = 0;
	Point s_config = NodeIDtoConfiguration((float)S);
	Point g_config = NodeIDtoConfiguration((float)G);
	std::complex<float> diff (s_config.x-g_config.x,s_config.y-g_config.y);
	dist = std::norm(diff);
	return dist;
}

void Planner::GetSuccessors(int nid, std::vector<float> &successors)
{
	Point coord = NodeIDtoGridCoord((float)nid);
	std::cout<<coord.x<<" "<<coord.y<<"\n";
	coord.x = coord.x ;
	coord.y = coord.y + 1;
	if(GridCoordtoNodeID(coord)>0 && InBoundary(GridCoordtoConfiguration(coord)) && !InCollision(GridCoordtoConfiguration(coord))){ successors.push_back(GridCoordtoNodeID(coord));}
	coord.x = coord.x;
	coord.y = coord.y - 2; // = y-1
	if(GridCoordtoNodeID(coord)>0 && InBoundary(GridCoordtoConfiguration(coord)) && !InCollision(GridCoordtoConfiguration(coord))) { successors.push_back(GridCoordtoNodeID(coord));}
	coord.x = coord.x + 1;
	coord.y = coord.y+1;     // = y
	if(GridCoordtoNodeID(coord)>0 && InBoundary(GridCoordtoConfiguration(coord)) && !InCollision(GridCoordtoConfiguration(coord))) { successors.push_back(GridCoordtoNodeID(coord));}
	coord.x = coord.x -2 ;
	coord.y = coord.y;     // = y
	if(GridCoordtoNodeID(coord)>0 && InBoundary(GridCoordtoConfiguration(coord)) && !InCollision(GridCoordtoConfiguration(coord))) { successors.push_back(GridCoordtoNodeID(coord));}


}

void Planner::AstarPlan(std::vector<Point> &final)
{
	float goal_ = ConfigurationtoNodeID(goal);
	float start_ = ConfigurationtoNodeID(start);

	if(!InCollision(goal))	{
		std::cout<<"GOAL not in COLLISION "<<goal_;
	
		g_cost[start_] = 0;
		f_cost[start_] = g_cost[start_] + ComputeHeuristicCost((int)start_,(int)goal_);
		open_set.insert(start_);
		bool path_found;
		int count = 0;
//	while(count!=10)
		while(open_set.size()!=0)
		{
			float min_node = 0;
			float min_f_cost = 10000; //infinity
			
			for( std::set<float>::iterator it = open_set.begin();it!=open_set.end();it++)
			{
				if(f_cost[*it] < min_f_cost)
				{
					min_node = *it;
					min_f_cost = f_cost[*it];
					//std::cout<<"fcost: "<<min_f_cost<<std::endl;
				}
			}
		
			float current = min_node;
			open_set.erase(current);
			closed_set.insert(current);
			std::cout<<"Current Point :: "<<current<<std::endl;
			if(current == goal_)
			{
				std::cout<<"Found goal"<<std::endl; //\n"<<current<<" "<<goal_;
				int u ;
				u = current;
				std::cout<<u<<current<<goal_;
				std::cout<<" start"<<std::endl;//<<(float)u<<" "<<start_<<"done";
			
				std::vector<Point>::iterator it;
			
				it = final.begin();
				while((float)u!=(float)start_)
				{
					std::cout<<"Present Point : "<<u<<" Target point :  "<<start_<<std::endl;
				  it = final.insert ( it , NodeIDtoConfiguration(u) );
					//std::cout<<"Point Added in map\n";
 					u = node_map[u];
					std::cout<<"Next Point : "<<u<<::std::endl;
		
				}
				final.insert(it,start);
				it = final.end();
				final.insert(it,goal);
				path_found = true;
				std::cout<<final.size()<<std::endl;
				open_set.clear();
				
				std::cout<<open_set.size()<<std::endl;
				break;
	
			}
	
			std::vector<float> successors;
			GetSuccessors(current,successors);
			std::cout<<"successors\n";
			for(int i =0;i<successors.size();i++)
			{
				std::cout<<successors.at(i)<<std::endl;
			}
			
			std::cout<<"\nDEBUG planner\n";
			
			for(int i=0;i<successors.size();i++)
			{
				std::set<float>::iterator it = closed_set.find(successors.at(i));
				float temp_g_cost;
				//std::cout<<*it<<"\n";
				if(it==closed_set.end())
				{
					std::cout<<"not there\n";
					temp_g_cost = g_cost[current] + ComputeDistance(current,successors.at(i));
					std::set<float>::iterator it_ = open_set.find(successors.at(i));
					if(it_==open_set.end() || temp_g_cost< g_cost[successors.at(i)])
					{
						node_map[successors.at(i)] = current;
						g_cost[successors.at(i)] = temp_g_cost;
						f_cost[successors.at(i)] = g_cost[successors.at(i)] + ComputeHeuristicCost(successors.at(i),goal_);
						open_set.insert(successors.at(i));
						
					}
				}
			}
			count++;
			std::cout<<"open_set size "<<open_set.size()<<std::endl;
		}
		if(!path_found && !open_set.size()&0) std::cout<<"Path NOT found\n";
		else std::cout<<"Path Found";//<<std::endl;
		//ShortenPlan(final);
		//ShortenPlan(final);
	}
	else std::cout<<"Goal in Collision and Path NOT found"<<std::endl;
}

void Planner::ShortenPlan(std::vector<Point> &final)
{
	//std::map<Point,Point> node_pair;
	std::vector<Point> copy_final = final;
	Point next,parent,compare;
//	copy_final.push_back(final.at(0));
	std::vector<Point>::iterator it;
	
	parent = final.at(0);
	compare = parent;
	next = final.at(2);

	int count=0;	
	int copy_count = 0;
	it = final.begin();
	
	//for(int i=0;i<final.size();i++){
	while(ConfigurationtoNodeID(compare)!=ConfigurationtoNodeID(copy_final.at(copy_final.size()-1))){
		//node_pair[final.at(i)] = final.at(i+1);
		int ob_count = 0;
		for(int o = 0 ; o<ob.size(); o++){
			int k=0,j=0;
			do{
	        int k = (j+1)%4;
					if(!doIntersect(ob[o].p[j], ob[o].p[k], parent, next)){
						ob_count++;
					}
				j = k;
			}
			while(j!=0);
		}
		std::cout<<ob_count<<std::endl;
		
		if(ob_count==ob.size()*4) {
			it = final.begin()+count+1;
			final.erase(it);
			parent = final.at(count);
			next = final.at(count+2);
			it = final.begin() + count + 2;
			std::cout<<"erased\n";
		}
		else{
			parent = copy_final.at(copy_count);
			next = copy_final.at(copy_count+2);
			it = copy_final.begin() + copy_count+2;
			count++;
			std::cout<<"centre changed"<<count;
		}
		compare = final.at(count +2);
		copy_count++;
		std::cout<<"plan size "<<final.size();
	}
	std::cout<<"Plan Shortening done. Plan size :"<<final.size();
}

void Planner::LocalPlan(std::vector<Point> &final)
{
	Point start = ConfigurationtoGridCoord(final.at(0));
	Point next = ConfigurationtoGridCoord(final.at(1));
	std::vector<Point> local_plan;
	local_plan.push_back(final.at(0));
	
	bool horizontal, vertical;	

	if(fabs(start.y-next.y)==1) horizontal = true;
	if(fabs(start.x-next.y)==1) vertical = true;
	
	for(int i=1;i<final.size();i++)
	{
		next = ConfigurationtoGridCoord(final.at(i));
		
		if(fabs(start.y-next.y)==1)
		{
			if(vertical==true) {
				local_plan.push_back(final.at(i-1));
				start = ConfigurationtoGridCoord(final.at(i-1));
				vertical =false;
				horizontal = true;
			}
		}
		if(fabs(start.x-next.x)==1)
		{
			if(horizontal==true) {
				local_plan.push_back(final.at(i-1));
				start = ConfigurationtoGridCoord(final.at(i-1));
				vertical = true;
				horizontal = false;
			}	
		}
	}
	final = local_plan;
	
	std::cout<<"Local Plan size "<<local_plan.size();
}

void Planner::SendPlan(std::vector<Point> &getPlan,std::vector< std::vector<Point> > &sendPlan)
{
	int count = 0;
	for(std::vector<Point>::iterator it = getPlan.begin();it!=getPlan.end()-1;it++){
		std::vector<Point> node;
		node.push_back(*it);
		node.push_back(*(it+1));
		std::cout<<getPlan.at(count).x<<" "<<getPlan.at(count+1).x<<std::endl;
		sendPlan.push_back(node);
		count++;
	}
}

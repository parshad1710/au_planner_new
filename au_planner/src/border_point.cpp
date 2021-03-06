#include <ros/ros.h>

#include <au_planner_msgs/Boundary.h>
#include <std_msgs/String.h>

ros::Publisher pub_border_points ;
ros::Publisher pub_planner;

int main(int argc,char **argv)
{
	ros::init(argc,argv,"border_point");
	ros::NodeHandle n;
	
	ros::Rate r(1);

	au_planner_msgs::Boundary boundary;
	
	pub_border_points = n.advertise<au_planner_msgs::Boundary>("get_boundary",10);		
	pub_planner = n.advertise<std_msgs::String>("get_planner",10);
	
	float o_points[] = {2,2,2,3,3,3,3,2,3.5,3.5,3.5,4,4,4.5,4.5,3.5};
	
	for(int i=0;i<2;i++)
	{
		for(int j=0;j<8;j++)
		{
			geometry_msgs::Point p;
			p.x = o_points[(i*8+j)];
			p.y = o_points[(i*8+j+1)];
			p.z = 0;
			boundary.obstacles.push_back(p);
			j++;
		}
	}
	
	boundary.start.x = 0.9;
	boundary.start.y = 1;
	boundary.goal.x = 3.6;
	boundary.goal.y = 4.5;
	
	boundary.upper.x = 5;
	boundary.upper.y = 5;
	boundary.lower.x = 0;
	boundary.lower.y = 0;
	
	boundary.resolution = (float)0.2;
	std_msgs::String st;
	st.data = "Astar";	

	while(ros::ok())
	{
		pub_border_points.publish(boundary);
		pub_planner.publish(st);
	}
	ros::spin();
}

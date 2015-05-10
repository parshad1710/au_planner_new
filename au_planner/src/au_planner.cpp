#include <ros/ros.h>

#include <au_planner/planner.h>
#include <au_planner/obstacle.h>
#include <geometry_msgs/Point.h>
#include <au_planner_msgs/Boundary.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <string>

float g_resolution;
Point g_start,g_goal,g_lower,g_upper;
bool g_pub_rviz=false;
bool g_pub_plan=false;
bool g_send_plan = false;
geometry_msgs::Point g_p,g_pub_robot;
std::string g_planner_name;
std::vector< std::vector<Point> > g_send_map;
int g_feed_count = 0;


visualization_msgs::Marker g_map,g_obstacles,g_robot,g_point,g_plan;

Planner *g_doPlan = new Planner(g_resolution,g_upper,g_lower,g_start,g_goal);
Obstacle lines;

ros::Publisher pub_rviz;
ros::Publisher pub_robot;

void visualizationInit()
{
	//visualization_msgs::Marker g_map,g_bstacle,g_robot,g_point,g_plan;
	
	g_map.header.frame_id = g_obstacles.header.frame_id = g_robot.header.frame_id = g_point.header.frame_id = g_plan.header.frame_id = "/my_frame";
	
	g_map.header.stamp = g_obstacles.header.stamp = g_robot.header.stamp = g_point.header.stamp = g_plan.header.stamp = ros::Time::now();
	
	g_map.ns = g_obstacles.ns = g_robot.ns = g_point.ns = g_plan.ns = "g_m_o_r_p";
	
	g_map.action = g_obstacles.action = g_robot.action = g_point.action = g_plan.action = visualization_msgs::Marker::ADD;
	
	g_map.pose.orientation.w = g_obstacles.pose.orientation.w = g_robot.pose.orientation.w = g_point.pose.orientation.w = g_plan.pose.orientation.w = 1.0;

	g_map.id = 0;
	g_robot.id = 1;
	g_obstacles.id = 2;
	g_point.id = 3;
	g_plan.id = 4;

	g_map.type = visualization_msgs::Marker::LINE_STRIP;
	g_obstacles.type = visualization_msgs::Marker::LINE_LIST;
	g_point.type = visualization_msgs::Marker::POINTS;
  g_robot.type = visualization_msgs::Marker::ARROW; //or MESH_RESOURCE
	g_plan.type = visualization_msgs::Marker::LINE_STRIP;

  //SCALE
	g_map.scale.x = 0.05;
	g_obstacles.scale.x = 0.02;
	g_point.scale.x = 0.05;
	g_point.scale.y = 0.05;
	g_robot.scale.x = 0.2;
	g_robot.scale.y = 0.05;
	g_robot.scale.z = 0.05;
	g_plan.scale.x = 0.02;


  //COLOR
	g_map.color.b = 1.0;
	g_map.color.a = 1.0;
	g_obstacles.color.r = 1.0;
	g_obstacles.color.a = 1.0;
	g_point.color.g = 1.0;
	g_point.color.a = 1.0;
	g_robot.color.b = 0.5;
	g_robot.color.g = 1;
	g_robot.color.a = 0.8;
	g_plan.color.b = 0.5;
	g_plan.color.g = 1;
	g_plan.color.r = 0.5;
	g_plan.color.a = 0.9;
}


void addLine(float x,float y,float z,visualization_msgs::Marker &object)
{
	g_p.x = x;
	g_p.y = y;
	g_p.z = z;
	object.points.push_back(g_p);
}

void showMap(Point l_l,Point u_l,visualization_msgs::Marker &object)
{
	addLine(l_l.x,l_l.y,0,object);
	addLine(l_l.x,u_l.y,0,object);
	addLine(u_l.x,u_l.y,0,object);
	addLine(u_l.x,l_l.y,0,object);
	addLine(l_l.x,l_l.y,0,object);	
}


void showObstacles(Obstacles ob,visualization_msgs::Marker &object)
{
	for(int i=0;i<4;i++){
		addLine(ob.p[i%4].x,ob.p[i%4].y,0,object);
		addLine(ob.p[(i+1)%4].x,ob.p[(i+1)%4].y,0,object);
	}
}

void showPlan(std::vector<Point> &map)
{
	for(int i=0;i<map.size();i++)
	{
		addLine((map.at(i)).x,(map.at(i)).y,0,g_plan);
	}
}

void pointPrint()
{
	if(g_pub_rviz){
		std::cout<<std::endl;
		std::cout<<"___________\n";
		std::cout<<"Upper "<<g_upper.x<<" "<<g_upper.y<<std::endl;
		std::cout<<"Lower "<<g_lower.x<<" "<<g_lower.y<<std::endl;
		std::cout<<"Number of Obstacles "<<g_doPlan->ob.size();
		std::cout<<"\n___________"<<std::endl;
	}
	if(g_pub_plan){
		std::cout<<std::endl;
		std::cout<<"___________\n";
		std::cout<<"Planner selected: "<<g_planner_name;
		std::cout<<"\n___________"<<std::endl;
	}
}

//void SendNode()
//{
	//geometry_msgs::Point 
//}

void RobotFeedback(const geometry_msgs::Point::ConstPtr& robot_feedback)
{
	//robot_feedback->x = centroid x
	//robot_feedback->y = centroid y
	//robot_feedback->z = angle of wrt to X axis (0 to 360)
	
	std::cout<<"publish";
	if(g_send_plan){
		std::vector<Point> node = g_send_map.at(g_feed_count);
		float diff_x = node.at(1).x-robot_feedback->x;
		float diff_y = node.at(1).x-robot_feedback->y;
		float diff_angle = atan2(diff_y,diff_x)*180/3.141592;
		
		diff_angle = 180-diff_angle;
//		diff_angle = (robot_feedback->z+(180-diff_angle))%360;
		
		int det_quad = floor((robot_feedback->z)/90);

		if(det_quad == 0){
			diff_angle = -(robot_feedback->z + diff_angle); //Clockwise
		}
		else if(det_quad == 4 || det_quad == 3){
			float sum_angles = diff_angle + robot_feedback->z;
			if((sum_angles)/360 > 0){
				diff_angle = -int(sum_angles)%360; //Clockwise
			}
			else{
				diff_angle = 360 - sum_angles;  //Counter-Clockwise
			}

		}
		
		if(diff_x<g_resolution/3 && diff_y<g_resolution/3){
			g_feed_count++;
		}
		else{
			if(fabs (diff_angle)>5){
				g_pub_robot.z = diff_angle; //in degrees
				g_pub_robot.x = 0;
				g_pub_robot.y = 0;
			}
			else{
				g_pub_robot.z = 0;
				g_pub_robot.x = 1;
				g_pub_robot.y = 1;
			}
		}
	}
}


void GetPlanner(const std_msgs::String::ConstPtr& planner_name)
{
	if(!g_pub_plan){
		g_planner_name = planner_name->data;
		std::vector<Point> final_map;
		if(planner_name->data=="Astar"){
			pointPrint();
			std::cout<<planner_name->data.c_str();
			g_doPlan->AstarPlan(final_map);
			g_doPlan->LocalPlan(final_map);
			g_pub_plan=true;
			showPlan(final_map);
			g_doPlan->SendPlan(final_map,g_send_map);
			g_send_plan = true;
		}
		//showPlan(final_map);
		std::cout<<"done";
	}
}

void GetBoundary(const au_planner_msgs::Boundary::ConstPtr& boundary_points)
{
	if(!g_pub_rviz){
		std::cout<<"debug_GB\n";
		g_lower.x = boundary_points->lower.x;
		g_lower.y = boundary_points->lower.y;
		g_upper.x = boundary_points->upper.x;
		g_upper.y = boundary_points->upper.y;
		showMap(g_lower,g_upper,g_map);
		
		geometry_msgs::Point p;
		g_p.x = boundary_points->goal.x;
		g_p.y = boundary_points->goal.y;
		g_p.z = 0;
		g_point.points.push_back(g_p);
		g_p.x = boundary_points->start.x;
		g_p.y = boundary_points->start.y;
		g_point.points.push_back(g_p);
		
		g_resolution = boundary_points->resolution;
		g_start.x = boundary_points->start.x;
		g_start.y = boundary_points->start.y;
		g_goal.x = boundary_points->goal.x;
		g_goal.y = boundary_points->goal.y;

		Obstacles ob[boundary_points->obstacles.size()/4];
		
		if((boundary_points->obstacles.size())%4 == 0){
			Planner *Map = new Planner(g_resolution,g_lower,g_upper,g_start,g_goal);
			g_doPlan = Map;
			for(int i=0;i<boundary_points->obstacles.size()/4;i++){
				for(int j=0;j<4;j++){
					ob[i].p[j].x = boundary_points->obstacles[i*4+j].x;
					ob[i].p[j].y = boundary_points->obstacles[i*4+j].y;
				}
				showObstacles(ob[i],g_obstacles);	
				g_doPlan->AddObstacles(ob[i]);
			}
		}
		else{
			std::cout<<"\nObstacle points not enough. Put rectangular obstacles only with 4 corner points\n";
		}
		std::cout<<"Points Fetching done";
		g_pub_rviz=true;
		pointPrint();
	}
}


void PubPoints()
{
	pub_rviz.publish(g_map);
	pub_rviz.publish(g_obstacles);
	pub_rviz.publish(g_robot);
	pub_rviz.publish(g_point);
	pub_rviz.publish(g_plan);
	pub_robot.publish(g_pub_robot);
}


int main(int argc, char ** argv)
{
	ros::init(argc,argv,"au_planner_node");	
	ros::NodeHandle n;
	
	pub_rviz = n.advertise<visualization_msgs::Marker>("visualization_marker",10);
	pub_robot = n.advertise<geometry_msgs::Point>("ar_nodes",10);

	ros::Subscriber sub_get_planner = n.subscribe<std_msgs::String>("get_planner",10,GetPlanner);
	ros::Subscriber sub_get_boundary = n.subscribe<au_planner_msgs::Boundary>("get_boundary",10,GetBoundary);
	ros::Subscriber sub_robot_feeback = n.subscribe<geometry_msgs::Point>("get_robot_feedback",10,RobotFeedback);
	
	ros::Rate r(10);

	visualizationInit();

	while(ros::ok()){
		PubPoints();
		ros::spinOnce();
		r.sleep();
	}
	ros::spin();

	return 0;
}

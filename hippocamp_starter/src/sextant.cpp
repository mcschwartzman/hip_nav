#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/PoseStamped.h"

// 0. Show only unexplored area
// 1. Receive new navgoal
// 2. Show new navgoal
// 3. Show nodes to be explored next
// 4. Show visited nodes 

// Nodes to show: unexplored, frontier (to be explored next), visited (explored nodes), goal

void newGoalCallback(const geometry_msgs::PoseStamped msg);

int main(int argc, char **argv){

	ros::init(argc, argv, "sextant");

	ros::NodeHandle n;

	ros::Publisher unexplored_pub = n.advertise<nav_msgs::GridCells>("unexplored", 100);
	ros::Publisher goal = n.advertise<nav_msgs::GridCells>("goal", 100);

	ros::Subscriber goal_sub = n.subscribe("move_base_simple/goal", 100, newGoalCallback);
	
	ros::Rate loop_rate(100);

	nav_msgs::GridCells msg;

	int width = 20;
	int length = 20;
	msg.cells.resize(width*length);
	msg.header.frame_id = "base_link";
	msg.cell_width = 1.0;
	msg.cell_height = 1.0;


	int count = 0;

	while (ros::ok()){

		for(int x=0; x<width; x++){
			for(int y=0; y<length; y++){

				int num = (x + y * width);

				geometry_msgs::Point &point = msg.cells[x + y * width];
				point.x = x/1.0;
				point.y = y/1.0;
				point.z = 1.0;
				
			}
		}

		msg.header.seq = count;
		msg.header.stamp = ros::Time::now();
		// ROS_INFO("%d", sizeof(msg.cells)/sizeof(msg.cells[0]));

		// ROS_INFO("%d, %d, %d", msg.cells[0].x, msg.cells[0].y, msg.cells[0].z);

		unexplored_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}

void newGoalCallback(const geometry_msgs::PoseStamped msg){

	ROS_INFO("new goal set for: %f, %f", msg.pose.position.x, msg.pose.position.y);


}
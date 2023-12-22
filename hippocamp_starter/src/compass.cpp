#include <vector>

#include "headers/GridCell.h"
#include "headers/Map.h"
#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/PoseStamped.h"

// 0. Show only unexplored area
// 1. Receive new navgoal
// 2. Show new navgoal
// 3. Show nodes to be explored next
// 4. Show visited nodes 

// Nodes to show: unexplored, frontier (to be explored next), visited (explored nodes), goal

// Signatures
std::vector<GridCell> filledNodeVector(int size);
nav_msgs::GridCells defineGrid(int size);

class Compass{
public:
	Compass(ros::NodeHandle n){
		goal_sub = n.subscribe("/move_base_simple/goal", 100, &Compass::newGoalCallback, this);

		goal_pub = n.advertise<nav_msgs::GridCells>("/goal", 100);

	}

	void newGoalCallback(const geometry_msgs::PoseStamped goal){
		ROS_INFO("new goal set for: %f, %f", goal.pose.position.x, goal.pose.position.y);

		nav_msgs::GridCells goal_grid = defineGrid(20);

		geometry_msgs::Point &goal_point = goal_grid.cells[0];
		goal_point.x = goal.pose.position.x;
		goal_point.y = goal.pose.position.y;
		goal_point.z = 0.2;

		goal_pub.publish(goal_grid);
	}

private:
	ros::Subscriber goal_sub;
	ros::Publisher goal_pub;
};

int main(int argc, char **argv){

	ros::init(argc, argv, "compass");
	ros::NodeHandle node;

	Compass compass(node);
	std::vector<GridCell> nodelist = filledNodeVector(10);


	ros::Publisher unexplored_pub = node.advertise<nav_msgs::GridCells>("unexplored", 100);

	ros::Rate loop_rate(100);

	nav_msgs::GridCells unexplored_grid = defineGrid(20);

	int count = 0;

	for(int i = 0; i < nodelist.size(); i++){
	
		ROS_INFO("GridCell %d at %d, %d", i, nodelist[i].getX(), nodelist[i].getY());

		geometry_msgs::Point point;
		point.x = nodelist[i].getX();
		point.y = nodelist[i].getY();
		point.z = 0.1;

		unexplored_grid.cells.push_back(point);


	}

	while (ros::ok()){
		

		unexplored_grid.header.seq = count;
		unexplored_grid.header.stamp = ros::Time::now();

		unexplored_pub.publish(unexplored_grid);

		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}

nav_msgs::GridCells defineGrid(int size) {

	nav_msgs::GridCells msg;

	msg.cells.resize(size^2);
	msg.header.frame_id = "base_link";
	msg.cell_width = 1.0;
	msg.cell_height = 1.0;

	return msg;

}

std::vector<GridCell> filledNodeVector(int size) {

	std::vector<GridCell> mapvector;

	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			GridCell g(i, j);
			mapvector.push_back(g);
		}
	}

	return mapvector;

}
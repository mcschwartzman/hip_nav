#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/PoseStamped.h"



class Visualizer{
private:

	ros::Subscriber goal_sub;
	ros::Publisher goal_pub;

public:


	Visualizer(ros::NodeHandle n, int s){
		goal_sub = n.subscribe("/move_base_simple/goal", 100, &Navstack::newGoalCallback, this);
		goal_pub = n.advertise<nav_msgs::GridCells>("/goal", 100);

		frontier_sub = n.subscribe("/") //todo


	}

	void newExploredCallback(){

	}

	void newFrontierCallback(){

	}

	void newGoalCallback(const geometry_msgs::PoseStamped goal){

		// when i see a new goal, display it in rviz

		ROS_INFO("new goal set for: %f, %f", goal.pose.position.x, goal.pose.position.y);

		std::vector<GridCell*> goalMatches = getNodes(goal.pose.position.x, goal.pose.position.y);

		if(goalMatches.size() > 0){

			goalNode = goalMatches[0];

			ROS_INFO("goal at: %d", (int *)goalNode);
			ROS_INFO("goal at: %d", (int *)goalMatches[0]);

			ROS_INFO("number of neighbors: %d", goalNode->neighbors.size());

			nav_msgs::GridCells goal_grid = defineGrid(20);

			geometry_msgs::Point &goal_point = goal_grid.cells[0];
			goal_point.x = goalMatches[0]->getX();
			goal_point.y = goalMatches[0]->getY();
			goal_point.z = 0.2;

			goal_pub.publish(goal_grid);
		}

		else{
			ROS_INFO("no node found there");
		}
	}



};
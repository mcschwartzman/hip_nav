#include <vector>
#include <cmath>
#include <stack>

#include "headers/GridCell.h"
#include "ros/ros.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/PoseStamped.h"

// 0. Show only unexplored area
// 1. Receive new navgoal
//		a. Find closest node at that location
//		b. Display a cell at that node
// 2. Show new navgoal
// 3. Show nodes to be explored next
// 4. Show visited nodes 

// Nodes to show: unexplored, frontier (to be explored next), visited (explored nodes), goal

// Signatures
nav_msgs::GridCells defineGrid(int size);
int matches(std::vector<GridCell*> comb, GridCell* cell);
std::vector<GridCell*> captureParents(GridCell* g);

class Navstack{
private:
	int size;
	ros::Subscriber goal_sub;
	ros::Publisher goal_pub;

public:

	GridCell* goalNode;
	GridCell* startingNode;

	std::vector<GridCell> nodeList;
	std::vector<GridCell*> frontier;
	std::vector<GridCell*> path;

	std::vector<GridCell*> getNodes(int x, int y) {

		int roundedX = round(x);
		int roundedY = round(y);

		std::vector<GridCell*> matches;

		for (int i = 0; i < nodeList.size(); i++){

			if (nodeList[i].getX() == roundedX){
				if (nodeList[i].getY() == roundedY){
					matches.push_back(&nodeList[i]);
				}
			}
		}

		return matches;
	}

	void mapNode(int x, int y) {
		GridCell g(x, y);
		nodeList.push_back(g);
	}

	void connectNode(GridCell* g) {
		int x = g->getX();
		int y = g->getY();

		std::vector<GridCell*> norths = getNodes(x, y+1);
		if(norths.size() > 0){
			g->neighbors.push_back(norths[0]);
		}
		std::vector<GridCell*> souths = getNodes(x, y-1);
		if(souths.size() > 0){
			g->neighbors.push_back(souths[0]);
		}
		std::vector<GridCell*> easts = getNodes(x+1, y);
		if(easts.size() > 0){
			g->neighbors.push_back(easts[0]);
		}
		std::vector<GridCell*> wests = getNodes(x-1, y);
		if(wests.size() > 0){
			g->neighbors.push_back(wests[0]);
		}
	}

	std::vector<GridCell> potentialNeighbors(GridCell g){
		int x = g.getX();
		int y = g.getY();

		std::vector<GridCell> neighbors;

		return neighbors;

	}



	Navstack(ros::NodeHandle n, int s){
		goal_sub = n.subscribe("/move_base_simple/goal", 100, &Navstack::newGoalCallback, this);

		goal_pub = n.advertise<nav_msgs::GridCells>("/goal", 100);

		for (int i = 0; i < s; i++) {
			for (int j = 0; j < s; j++) {
				mapNode(i, j);
			}
		}

		for (int i = 0; i<nodeList.size(); i++){
			connectNode(&nodeList[i]);
		}

	}

	void newGoalCallback(const geometry_msgs::PoseStamped goal){

		startingNode = &nodeList[0];

		std::vector<GridCell*> goalMatches = getNodes(goal.pose.position.x, goal.pose.position.y);

		if(goalMatches.size() > 0){

			goalNode = goalMatches[0];

			ROS_INFO("start %d at %d, %d", (int *)startingNode, startingNode->getX(), startingNode->getY());
			ROS_INFO("goal %d at %d, %d", (int *)goalNode, goalNode->getX(), goalNode->getY());

			if(matches(frontier, goalNode) > 0){
				ROS_INFO("goal node in frontier");
			}

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

	void refresh(void){
		for (int i=0; i < nodeList.size(); i++){
			nodeList[i].visited = false;
		}
	}



};

int main(int argc, char **argv){

	ros::init(argc, argv, "compass");
	ros::NodeHandle node;

	Navstack ns(node, 20);

	ros::Publisher unexplored_pub = node.advertise<nav_msgs::GridCells>("unexplored", 100);
	ros::Publisher frontierPub = node.advertise<nav_msgs::GridCells>("frontier", 100);
	ros::Publisher pathPub = node.advertise<nav_msgs::GridCells>("path", 1);

	ros::Rate loop_rate(100);

	nav_msgs::GridCells unexplored_grid = defineGrid(20);
	nav_msgs::GridCells frontierGrid = defineGrid(20);
	nav_msgs::GridCells pathGrid = defineGrid(20);

	int messageCount = 0;
	int state = 0;
	int unexploredCount = ns.nodeList.size();

	GridCell* startingNode = &ns.nodeList[0];
	startingNode->visited = true;

	std::vector<GridCell*> stackVector;
	stackVector.push_back(startingNode);

	GridCell* currentNode = nullptr;

	while (ros::ok()){

		ROS_INFO("State: %d, stacksize: %d, path: %d, pgrid: %d", state, stackVector.size(), ns.path.size(), pathGrid.cells.size());

		int frontierCount = ns.frontier.size();
		int pathCount = ns.path.size();

		// State 0 - map nodes
		// artificial mapping phase
		// map nodes
		// connect nodes

		if (state == 0){

			if (unexploredCount > 0){

				// queueing unexplored nodes to render
				geometry_msgs::Point point;
				point.x = ns.nodeList[unexploredCount-1].getX();
				point.y = ns.nodeList[unexploredCount-1].getY();
				point.z = 0.1;
				unexplored_grid.cells.push_back(point);
				unexploredCount--;
			}
			
			if (frontierCount > 0){

				geometry_msgs::Point fpoint;
				fpoint.x = ns.frontier[frontierCount - 1]->getX();
				fpoint.y = ns.frontier[frontierCount - 1]->getY();
				fpoint.z = 0.3;
				// ROS_INFO("frontierCount: %d", frontierCount);
				frontierGrid.cells.push_back(fpoint);
				frontierCount--;
			}

			if (pathCount > 0){
				geometry_msgs::Point ppoint;
				ppoint.x = ns.path[pathCount - 1]->getX();
				ppoint.y = ns.path[pathCount - 1]->getY();
				ppoint.z = 0.2;
				pathGrid.cells.push_back(ppoint);
				pathCount--;
			}

			state = 1;
		}

		// State 1 - explore mapped nodes

		if (state == 1){

			// start with a node (start node)
			// iterate/recur through neighbors
			// if a neighbor is the goal, hooray!
			// do first step but with neighbor

			if(ns.goalNode){

				if(!stackVector.empty()){
					
					currentNode = stackVector.front();
					if(currentNode == ns.goalNode){

						ns.goalNode = nullptr;
						ns.frontier.clear();
						stackVector.clear();

						startingNode->visited = true;
						stackVector.push_back(startingNode);
						frontierGrid = defineGrid(20);

						pathGrid = defineGrid(20);

						ns.refresh();

						// ONLY DO THIS AFTER GOAL IS FOUND 
						// at this point, walk back along all parents to generate path
						// add the current node to the path, if it has a parent,
						// make that parent the current node, so that it gets added to the path
						// repeat until current node is the starting node
						// ONLY DO THE ABOVE WHILE STACKVECTOR IS 

						state = 2;
					}

					else{

						stackVector.erase(stackVector.begin());

						ns.frontier.push_back(currentNode);

						for(int i = 0; i < currentNode->neighbors.size(); i++){

							GridCell* n = currentNode->neighbors[i];

							if (n->visited == false){
								n->visited = true;
								n->parent = currentNode;
								stackVector.insert(stackVector.begin(), n);
							}
						}
						state = 3;
					}

				}

			}

			else{
				
				state = 2;
			}

		}

		if (state == 2){
		// State 2 - pathfind to goal node

			if(currentNode != NULL){

				ns.path.push_back(currentNode);

				if(currentNode->parent){
					currentNode = currentNode->parent;
				}

			}

			state = 3;

		}

		// State 3 - publish all gridCells messages

		if (state == 3){

			unexplored_grid.header.seq = messageCount;
			unexplored_grid.header.stamp = ros::Time::now();

			unexplored_pub.publish(unexplored_grid);
			frontierPub.publish(frontierGrid);
			pathPub.publish(pathGrid);

			ros::spinOnce();
			loop_rate.sleep();
			++messageCount;

			state = 0;
		}

		

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

int matches(std::vector<GridCell*> comb, GridCell* cell){
	int toReturn = 0;

	for(int i = 0; i < comb.size(); i++){
		if (comb[i] == cell){
			toReturn ++;
		}
	}

	return toReturn;
}

std::vector<GridCell*> captureParents(GridCell* g){

	ROS_INFO("trying to capture parents");

	std::vector<GridCell*> p;
	p.push_back(g);

	ROS_INFO("PUSHING BACK WAS FINE");

	if(g->parent){
		ROS_INFO("adding parent");
		captureParents(g->parent);
	}
	else{
		ROS_INFO("DONE PATHING");

		return p;
	}

}
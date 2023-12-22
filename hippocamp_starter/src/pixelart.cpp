#include "ros/ros.h"
#include "nav_msgs/GridCells.h"

#include "math.h"

void fillGrid(int w, int l, nav_msgs::GridCells targetMsg);
void oneCell(int w, int l, nav_msgs::GridCells targetMsg);

int main(int argc, char **argv){

	ros::init(argc, argv, "pixelart");

	ros::NodeHandle n;

	ros::Publisher art_pub = n.advertise<nav_msgs::GridCells>("art", 100);
	ros::Rate loop_rate(100);

	nav_msgs::GridCells msg;

	int width = 10;
	int length = 10;
	msg.cells.resize(width*length);
	msg.header.frame_id = "base_link";
	msg.cell_width = 1.0;
	msg.cell_height = 1.0;


	int count = 0;

	while (ros::ok()){

		// fillGrid(width, length, msg);

		// for(int x=0; x<width; x++){
		// 	for(int y=0; y<length; y++){
		// 		int num = (x + y * width);
		// 		ROS_INFO("%d", num);
		// 		if(x == 0 || x == 9 || y == 0 || y == 9){
		// 			geometry_msgs::Point &point = msg.cells[x + y * width];
		// 			point.x = x/1.0;
		// 			point.y = y/1.0;
		// 			point.z = 1.0;
		// 		}
		// 	}
		// }

		for(int x=0; x<width; x++){
			for(int y=0; y<length; y++){

				int num = (x + y * width);

				ROS_INFO("%d", num);

				if(x+y != 18){
					geometry_msgs::Point &point = msg.cells[x + y * width];
					point.x = x/1.0;
					point.y = y/1.0;
					point.z = 1.0;
				}

				

				
			}
		}

		// geometry_msgs::Point point0;

		// geometry_msgs::Point &point = msg.cells[0];
		// point.x = 0.0;
		// point.y = 0.0;
		// point.z = 1.0;

		// geometry_msgs::Point &newpoint = msg.cells[99];
		// newpoint.x = 0.0;
		// newpoint.y = 9.0;
		// newpoint.z = 1.0;

		// msg.cells[99] = point0;
		// // msg.cells.push_back(point0);

		// geometry_msgs::Point point1;
		// point1.x = 0.0;
		// point1.y = 4.0;
		// point1.z = 0.0;

		// msg.cells[1] = point1;
		// // msg.cells.push_back(point1);

		// geometry_msgs::Point point2;
		// point2.x = 4.0;
		// point2.y = 4.0;
		// point2.z = 0.0;

		// msg.cells[2] = point2;
		// // msg.cells.push_back(point2);

		// geometry_msgs::Point point3;
		// point3.x = 4.0;
		// point3.y = 0.0;
		// point3.z = 0.0;

		// msg.cells[3] = point3;
		// msg.cells.push_back(point3);

		msg.header.seq = count;
		msg.header.stamp = ros::Time::now();

		ROS_INFO("%s", "publishing cells");
		// ROS_INFO("%d", sizeof(msg.cells)/sizeof(msg.cells[0]));

		// ROS_INFO("%d, %d, %d", msg.cells[0].x, msg.cells[0].y, msg.cells[0].z);

		art_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;

	}

	return 0;

}

void fillGrid(int w, int l, nav_msgs::GridCells targetMsg){

	for(int x=0; x<w; x++){
			for(int y=0; y<l; y++){
				geometry_msgs::Point &point = targetMsg.cells[x + y * w];
				point.x = x/1.0;
				point.y = y/1.0;
				point.z = 1.0;
			}
		}
}

void oneCell(int w, int l, nav_msgs::GridCells targetMsg){

	geometry_msgs::Point point0;
	point0.x = 0.0;
	point0.y = 0.0;
	point0.z = 1.0;

	targetMsg.cells.push_back(point0);

	geometry_msgs::Point point1;
	point1.x = 0.0;
	point1.y = 100.0;
	point1.z = 1.0;

	targetMsg.cells.push_back(point1);
}
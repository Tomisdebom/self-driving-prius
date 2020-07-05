#include <ros/ros.h>
#include <prius_msgs/Control.h>
#include <iostream>
#include <vector> 
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <prius_msgs/Control.h>
#include <stdlib.h>
#include <turtlesim/Pose.h>
#include <iomanip>

#include <boost/foreach.hpp>

#include <cmath>

#include "vision_msgs/Detection3DArray.h"
#include "vision_msgs/Detection3D.h"
#include "vision_msgs/Detection2DArray.h"
#include "vision_msgs/Detection2D.h"
#include "vision_msgs/ObjectHypothesisWithPose.h"
#include <sensor_msgs/PointCloud2.h>

class PriusControl{
    
	int check;

	ros::NodeHandle nh;

	ros::Subscriber sub_human = nh.subscribe("opencv_solution_node/detections", 1, &PriusControl::input_callback_human, this); 

	ros::Subscriber sub_object = nh.subscribe("pcl_solution_node/detections", 1, &PriusControl::input_callback_obstacle, this);

	ros::Publisher pub = nh.advertise<prius_msgs::Control>("prius", 1);

public:
// Callback function for human detection
void input_callback_human(const vision_msgs::Detection2DArray& human_msg)
{

	// Initialise message to be sent to the prius
  	prius_msgs::Control prius_msg;   

	// Loop through detections
  	for (int k = 0; k<human_msg.detections.size();k++){
    // Check whether the area of the detection is larger than 25000 pixels
    double pixels = human_msg.detections[k].bbox.center.x*human_msg.detections[k].bbox.center.y;
	if (pixels>25000){
		// If so, set brake and throttle so that the prius halts. Also set check factor to 1
		prius_msg.brake = 1;
		prius_msg.throttle = 0;
		check = 1;
		break;
	}

  }
  pub.publish(prius_msg);

}

// Callback function for obstacle detection
void input_callback_obstacle(const vision_msgs::Detection3DArray& obstacle_msg)
{
	// Initialise message to be sent to the prius
    prius_msgs::Control prius_msg;  

	// If checking factor is equal to 1: keep the streer and throttle at 0 
    if(check ==1){
		prius_msg.steer = 0;
		prius_msg.throttle = 0;
    }
	// Else, check and act on obstacles
    else{

    // Steer and move forwar until we reach an obstacle
  	prius_msg.steer = 0;
  	prius_msg.throttle = 1;
  
    // Initialize vectors containing obstacles distances and y positions
    std::vector<double> obstacle_distance;
    std::vector<double> y_position;

    // Check whether each detection is within 6 meters of the Priys
    for (int i = 0; i<obstacle_msg.detections.size();i++){
    	double xdist = obstacle_msg.detections[i].bbox.center.position.x;
    	double ydist = obstacle_msg.detections[i].bbox.center.position.y;
    	double distance_detection = sqrt(xdist*xdist+ydist*ydist);
    
		// If the obstacles is within 6m, put the distance in the distance vector y_position
		if (xdist > 0 & distance_detection < 6){
		obstacle_distance.push_back(distance_detection);
		y_position.push_back(ydist);
		}
    }
    // Take the closest obstacles and act accordanly
    if (obstacle_distance.size() > 0){
		int minElementIndex = std::min_element(obstacle_distance.begin(),obstacle_distance.end()) - obstacle_distance.begin();
		if (y_position[minElementIndex] >0){ 	
			// Steer right
			prius_msg.steer = -1;}
		else if (y_position[minElementIndex] <=0){ 
			// Steer left
			prius_msg.steer = 1;}
    }

	pub.publish(prius_msg);
	}
}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "publish_situation");
	prius_msgs::Control tvt;
	PriusControl ic;
	ros::spin();
	return 0;
}
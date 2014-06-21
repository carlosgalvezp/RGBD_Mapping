 /*
 * Human Interface2.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * part of the differential_drive ROS package, for ROBOT13@KTH
 * 
 * Draw an interface, enable keyboard control
 * Cool OpenCV features :)
 */

#include "ros/ros.h"
#include <string.h>
#include <algorithm>

#include "differential_drive/KeyEvent.h"
#include "differential_drive/MouseEvent.h"
#include "differential_drive/Pose.h"
#include "differential_drive/Path.h"
#include "differential_drive/Carrot.h"

#include "differential_drive/Odometry.h"
#include "differential_drive/Speed.h"
#include "std_msgs/Header.h"

#include "dimension.h"

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>

ros::Publisher key_pub;

ros::Subscriber pose_sub;
ros::Subscriber pose_target_sub;

#define WIDTH 600
#define HEIGHT 600

bool redraw = true;

float pose_x = 0;
float pose_y = 0;
float pose_theta = 0;

float pose_target_x = 0;
float pose_target_y = 0;
float pose_target_theta = 0;

// Receive pose estimate
void receive_pose_estimate(const differential_drive::Odometry& msg)
{
	pose_x = msg.x;
	pose_y = msg.y;
	pose_theta = msg.theta;
	redraw = true;
	printf("Estimated position is: (%.3fm, %.3fm) heading: %.1fdeg\n", pose_x, pose_y, pose_theta*180/M_PI);
}

// Receive pose target
void receive_pose_target(const differential_drive::Speed& msg)
{	
	static uint32_t time,time_old;
	static float Te;
	static float V_lin,V_rot;
		
	time_old = time ;
	time = msg.header.stamp.nsec ;
	if(time>time_old)	{
		Te = (time - time_old)*1E-9 ;
	}
	else	{
		Te = (1E9 + time - time_old)*1E-9;
	}
	
	V_lin = r/2*(msg.W1+msg.W2);
	V_rot = r/B*(msg.W1-msg.W2);
	
	pose_target_theta += V_rot*Te;
	pose_target_x += V_lin*Te*cos(pose_target_theta);
	pose_target_y += V_lin*Te*sin(pose_target_theta);
	
	redraw = true;
	printf("Target position is: (%.3fm, %.3fm) heading: %.1fdeg\n", pose_target_x, pose_target_y, pose_target_theta*180/M_PI);
}

void draw_robot(cv::Mat& screen, float x, float y, float theta, uint8_t r, uint8_t g, uint8_t b)
{
	static const float triangle_x[3] = {.2,-.1,-.1};
	static const float triangle_y[3] = {0,.1,-.1};
	float sx[3];
	float sy[3];

	// Calculate vertex positions
	float c = cos(theta);
	float s = sin(theta);
	for (int i = 0; i < 3; ++i)
	{
		float tx = c * triangle_x[i] + -s * triangle_y[i] + x;
		float ty = s * triangle_x[i] + c * triangle_y[i] + y;

		sx[i] = WIDTH/2 + round(tx*100); 
		sy[i] = HEIGHT/2 + round(ty*100); 

	}

	// Draw the loop
	for (int i = 1; i < 3; ++i)	{
		cv::line(screen, cv::Point(sx[i-1], sy[i-1]), cv::Point(sx[i],sy[i]), CV_RGB(r,g,b), 1, CV_AA);
	}
	cv::line(screen, cv::Point(sx[2], sy[2]), cv::Point(sx[0],sy[0]), CV_RGB(r,g,b), 1, CV_AA);

	// Point in center of robot
	

}

void DrawScreen(cv::Mat& screen)
{
	screen.setTo(0);
    // Draw target robot pose
    draw_robot(screen, pose_target_x, pose_target_y, pose_target_theta, 0,255,0);

    // Draw estimated robot pose
    draw_robot(screen, pose_x, pose_y, pose_theta, 0,0,255);
    
    cv::imshow("Human Interface", screen);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "HumanInterface"); //Creates a node named "HumanInterface"
	ros::NodeHandle n;

	pose_sub = n.subscribe("/motion/Odometry", 1000, receive_pose_estimate);
	pose_target_sub = n.subscribe("/motion/Speed", 1000, receive_pose_target);

	cv::Mat screen(HEIGHT,WIDTH,CV_8UC1,cv::Scalar(0));
	cv::cvtColor(screen, screen, CV_GRAY2BGR);
	
	//ros::Rate loop_rate(30); // frequency
	cv::imshow("Human Interface", screen);
	cv::waitKey(1);
	
	int time_old = ros::Time::now().nsec;
	
	while (ros::ok()) {
		if (ros::Time::now().nsec-time_old > 100E6	&& redraw) {
			time_old = ros::Time::now().nsec ;
			DrawScreen(screen);
			redraw = false;
		}

		//loop_rate.sleep(); // Sleep for 10 ms
		ros::spinOnce(); // Process ros messages
	}

	return EXIT_SUCCESS;
}


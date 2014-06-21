 /*
 * openLoop.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * part of the differential_drive ROS package, for ROBOT13@KTH
 * 
 * Example of how to use openLoop mode
 * 		- Send speed instruction (W1,W2)=(0,0)
 * 		- Read data from the encoders
 * 		
 * 	Please note that the speed instructions are the duty cycle of the PWM
 * 	So, it *SHOULD* be between 0 and 255
 * 
 */

#include "ros/ros.h"

/* Include specific messages types */
#include "differential_drive/Speed.h"
#include "differential_drive/Encoders.h"

#include <iostream>

differential_drive::Speed consigne;

/* Callback in "/motion/Encoders" topic */
void Callback(const differential_drive::Encoders& msg)
{
  ROS_INFO("I heard: %i %i %i", msg.delta_encoder1,msg.delta_encoder2,msg.timestamp);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "openLoop_example");

  ros::NodeHandle n;

  /* Advertise on "/motion/Speed" */
  ros::Publisher pub = n.advertise<differential_drive::Speed>("/motion/Speed", 1000);
  
  /* Advertise on "/motion/Encoders". Callback fct will be called */
  ros::Subscriber sub = n.subscribe("/motion/Encoders", 1000, Callback);

  /* 10 Hz rate */
  ros::Rate loop_rate(10);
  
  /* No motion */
  consigne.W1 = 0 ;
  consigne.W2 = 0 ;
  
  while(ros::ok())	{
	  pub.publish(consigne);
	  ros::spinOnce(); 
	  loop_rate.sleep();
  }

  return 1;
}

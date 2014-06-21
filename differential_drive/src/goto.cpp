 /*
 * goto.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * part of the differential_drive ROS package, for ROBOT13@KTH
 * 
 * Send the same angular position to all servomotors
 * 		Input parameter : @int angle, in [0,180]
 * 
 * 
 */

#include "ros/ros.h"

#include "differential_drive/Servomotors.h"

#include <iostream>

differential_drive::Servomotors msg;

int main(int argc, char **argv)
{
	
  ros::init(argc, argv, "no_name");

  ros::NodeHandle n;

  /* Set up publisher on "/actuator/Servo" topic */
  ros::Publisher pub = n.advertise<differential_drive::Servomotors>("/actuator/Servo", 1000);
  
  ros::Rate loop_rate(1);
  
  /* if not parameter given */
  if(argc<2)	{
	  printf("Missing argument : angle \n");
	  return 1;
  }
  
  /* Set the same position for each servo */
  for(int i=0;i<8;i++)	{
  		  msg.servoangle[i] = (char)atoi(argv[1]);  
  } 
  
  /* Hack to send the position : first attempt to publish does not seem to work. Need to reiterate */
  /* TODO : understand why */
  for(int k=0;k<2;k++)	{
	  pub.publish(msg); 
	  loop_rate.sleep();
  }
  
  printf("Asked to go to %i degrees \n",msg.servoangle[0]);

  return 0;
}

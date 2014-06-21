 /*
 * mode2.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * part of the differential_drive ROS package, for ROBOT13@KTH
 * 
 * Read ADC values and convert in meters
 */

#include "ros/ros.h"
#include "differential_drive/Speed.h"
#include "differential_drive/Encoders.h"
#include "differential_drive/Odometry.h"
#include <differential_drive/AnalogC.h>

#include "std_msgs/Header.h"

#include "differential_drive/Sharp.h"
#include <iostream>
#define N 10

differential_drive::Speed consigne;

void chatterCallback(const differential_drive::Speed& msg)
{
 ROS_INFO("W1 : %f    W2 : %f", msg.W1,msg.W2);
}

void sharpCallback(const differential_drive::AnalogC& adc_msg)
{
static int voie1[N],voie2[N],voie3[N],voie4[N];
static float loc1,loc2,loc3,loc4;
static int k=0;
static float d1,d2,d3,d4;
voie1[k%N]=adc_msg.ch1;
voie2[k%N]=adc_msg.ch2;
voie3[k%N]=adc_msg.ch3;
voie4[k%N]=adc_msg.ch4;
k++;
  for(int i=0;i<N;i++)	{
loc1 += voie1[i];
loc2 += voie2[i];
loc3 += voie3[i];
loc4 += voie4[i];
  }

loc1 = loc1/N;
loc2 = loc2/N;
loc3 = loc3/N;
loc4 = loc4/N;

if(loc1 > 90 && loc1 < 600)	{
d1 = 1/(3.318E-5*loc1+2.8E-4);
}
else d1 = 0;

if(loc2 > 90 && loc2 < 600)	{
d2 = 1/(3.318E-5*loc2+2.8E-4);
}
else d2 = 0;

if(loc3 > 90 && loc3 < 600)	{
d3 = 1/(3.318E-5*loc3+2.8E-4);
}
else d3 = 0;

if(loc4 > 90 && loc4 < 600)	{
d4 = 1/(3.318E-5*loc4+2.8E-4);
}
else d4 = 0;
 
printf("%.2f %.2f %.2f %.2f \n", d1,d2,d3,d4);

loc1=0;
loc2=0;
loc3=0;
loc4=0;

ROS_INFO("Voltage : %f %f %f",adc_msg.cell1,adc_msg.cell2,adc_msg.cell3);
}

void odometryCallback(const differential_drive::Odometry& msg)
{
  ROS_INFO("I heard: %f %f %f", msg.x,msg.y,msg.theta);
}

void Callback(const differential_drive::Encoders& msg)
{
  ROS_INFO("I heard: %i %i %i", msg.delta_encoder1,msg.delta_encoder2,msg.timestamp);
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "node_recepteur");

  ros::NodeHandle n;

  
  ros::Publisher pub = n.advertise<differential_drive::Speed>("/motion/Speed", 1000);
  
  //ros::Subscriber sub = n.subscribe("speed", 1000, chatterCallback);

  //ros::Subscriber sub1 = n.subscribe("/motion/Odometry", 1000, sharpCallback);
  
  ros::Subscriber sub3 = n.subscribe("/motion/Encoders", 1000, Callback);
 
 //ros::Subscriber sub2 = n.subscribe("/motion/Odometry", 1000, odometryCallback);

  ros::Rate loop_rate(10);
  
  consigne.W1 = 0 ;
  consigne.W2 = 255 ;
  
  while(ros::ok())	{
	  pub.publish(consigne);
	  ros::spinOnce(); 
	  loop_rate.sleep();
	  ROS_INFO("Send : %f %f %f", consigne.W1,consigne.W2);
  }
  
    
  

  return 1;
}

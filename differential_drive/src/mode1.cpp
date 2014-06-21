 /*
 * mode1.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * part of the differential_drive ROS package, for ROBOT13@KTH
 * 
 * Run a UMBenchmark test 
 * Example of how to generate trapezoidal speed law
 * 
 */

#include "ros/ros.h"
#include "differential_drive/Speed.h"

#include "differential_drive/KeyEvent.h"

#include <math.h>
#include <string.h>

#include "dimension.h"

#include <iostream>


void trapeze(float d);
void rotation(float alpha);

float time_loc = 0.;

const int f = 50; 

const float A_max = r*3.;
const float V_max = r*6.;

const float W_max = 2.*r/B*3. ;
const float dW_max = 2.*r/B*0.5 ;

bool flag_new_trap = false ;
bool flag_new_rot = true ;
bool finish = false ;

differential_drive::Speed msg;

int main(int argc, char **argv)
{
	float a=90,d=1;
	int state=1;
 	ros::init(argc, argv, "node");
	
	ros::NodeHandle n;

	ros::Rate loop_rate(f);
	
	ros::Publisher send_speed = n.advertise<differential_drive::Speed>("/motion/Speed", 1000);
	
	msg.W1 = 0;
	msg.W2 = 0;
	
	for(int i=0;i<2*f;i++)	{
			send_speed.publish(msg);
			loop_rate.sleep(); 
	}
	

	while(ros::ok())
	{	
		msg.header.stamp.nsec = ros::Time::now().nsec;
		time_loc+=1./f;

		switch(state%2) {
		case 0 :
		  trapeze(d);
		  break;
		case 1 :
		  rotation(a*3.1415/180);
		  break;
		}
		  
		if(finish)	{
		  state++;
		  if(state%2){
		    flag_new_rot = true;
		  }
		  else{
		    flag_new_trap = true;
		  }
		  time_loc=0;
		  finish=false;
		}
		send_speed.publish(msg);
		if(state==9) {state=-1;ROS_INFO("finish");}
		//ROS_INFO("W1 : %f W2 : %f",msg.W1,msg.W2);
		ros::spinOnce();
		loop_rate.sleep(); 
			
	}

	return 0;
}

void trapeze(float d)  {
	static float eta = 0 ;
	static float V_lin_cons;
	if(flag_new_trap)	{
		if(fabs(d)<pow(V_max,2)/A_max)	{
			eta = sqrt(4*fabs(d)/A_max);
		}
		else	{
			eta = fabs(d)/V_max + V_max/A_max;
		}
		flag_new_trap = false;
		ROS_INFO("Will move for %.2f m \t ETA : %f s",d,eta);
		time_loc = 0;
	}
	if(eta != 0)	{
	    float V_lin_cons = fmin(V_max,fmin(A_max*time_loc,A_max*(eta-time_loc)));
	    if(d<0) {V_lin_cons = -V_lin_cons;}
	   	if(time_loc >= eta)  {V_lin_cons=0;eta=0;finish = true;}
	   	msg.W1 = 1./r*V_lin_cons;
	   	msg.W2 = msg.W1;
	}
	
}

void rotation(float alpha)  {
	static float eta = 0 ;
	static float W_rot_cons;
	if(flag_new_rot)	{
		if(fabs(alpha)<pow(W_max,2)/dW_max)	{
			eta = sqrt(4*fabs(alpha)/dW_max);
		}
		else	{
			eta = fabs(alpha)/W_max + W_max/dW_max;
		}
		ROS_INFO("Will turn for %.2f deg \t ETA : %f s",alpha*180/3.1415,eta);
		flag_new_rot = false;
	}
	if(eta != 0)	{
	    	float W_rot_cons = fmin(W_max,fmin(dW_max*time_loc,dW_max*(eta-time_loc)));
    		if(alpha<0) {W_rot_cons = -W_rot_cons;}
    		if(time_loc > eta)  {W_rot_cons=0;eta=0;finish = true;}
    		msg.W1 = B/(2*r)*W_rot_cons;
    		msg.W2 = -msg.W1;
	}
	
}


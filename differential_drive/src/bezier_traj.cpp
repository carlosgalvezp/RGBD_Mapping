 /*
 * bezier.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * part of the differential_drive ROS package, for ROBOT13@KTH
 * 
 * Send to the arduino board velocities for a bezier curve
 */



#include "ros/ros.h"
#include "differential_drive/Speed.h"
#include "Bezier.h"
#include "dimension.h"
#include <math.h>

const int f = 100 ;

float x_0 = 0.;
float y_0 = 0.;
float theta_0 = 0.;

float x_1 = 1.;
float y_1 = 1.;
float theta_1 = 0.;

bool run = false ;
bool new_traj = false ;

Bezier my_curve;

int main(int argc, char **argv)
{
 
	  ros::init(argc, argv, "Bezier");
	
	  ros::NodeHandle n;
	
	  ros::Publisher chatter_pub = n.advertise<differential_drive::Speed>("/motion/Speed", 1000);
	
	  ros::Rate loop_rate(f);
	  
	  differential_drive::Speed msg;

		x_1 = 1./100*(float)atoi(argv[1]);
		y_1 = 1./100*(float)atoi(argv[2]);
		theta_1 = 3.1415/180*(float)atoi(argv[3]);
	
		ROS_INFO("Let's compute control points !");
		if(!my_curve.set_control_points(0,0,0,x_1,y_1,theta_1))	{ROS_INFO("Done");}
		else{ROS_INFO("Error");return 1;}	
		ROS_INFO("Let's compute the curve !");
		if(!my_curve.compute_curve(r*2.,r*2.))	{
			ROS_INFO("Done");
			ROS_INFO("ETA : %f",my_curve._eta);		
		}
		else{ROS_INFO("Error");return 1;}
	
		float time_step = my_curve._eta/500;
		int i=0;
		int k=0 ;
		float time_loc = 0;
		float V1,V2,alpha;
		
		int start_time = ros::Time::now().sec ;

		msg.W1 = 0;
	    msg.W2 = 0;

		for(int i=0;i<f;i++)	{
			chatter_pub.publish(msg);
			loop_rate.sleep(); 
		}




  while (ros::ok())
  {	  	
	  	
	    msg.header.stamp.nsec = ros::Time::now().nsec;
	    
			k = floor(time_loc/time_step);
			
			V1 = my_curve.V_l[k];
			V2 = my_curve.V_l[k+1];
			alpha = (V2-V1)/time_step;
	
			msg.W1 = 1./r*(V1 + alpha*(time_loc-my_curve.t[k]));
	
			V1 = my_curve.V_r[k];
			V2 = my_curve.V_r[k+1];
			alpha = (V2-V1)/time_step;
	
			msg.W2 = 1./r*(V1 + alpha*(time_loc-my_curve.t[k]));		
	
			//ROS_INFO("Set W1 : %f    W2 : %f", msg.W1,msg.W2);
			//printf("%f %f %i\n", msg.W1,msg.W2,k);
	
			if(time_loc>my_curve._eta)	{
				ROS_INFO("Curve followed in %f \n",time_loc);
				return 0;
	
			}

			
			time_loc += 1./f ;
	    
	    
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();	

  }


  return 0;
}

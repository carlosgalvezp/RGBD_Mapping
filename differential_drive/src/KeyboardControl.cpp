 /*
 * KeyBoard.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * part of the differential_drive ROS package, for ROBOT13@KTH
 * 
 * Use Keyboard to control the robot
 */

#include "ros/ros.h"
#include "differential_drive/Speed.h"
#include "differential_drive/Lights.h"
#include "differential_drive/Encoders.h"

#include "differential_drive/KeyEvent.h"
#include "differential_drive/MouseEvent.h"

#include "std_msgs/Header.h"

#include <math.h>
#include <string.h>
#include <SDL/SDL.h>

#include <iostream>

using namespace differential_drive;
const float r = 48E-3; // wheel radius [m]
const float B = 232E-3;// baseline [m]

const float V_max = 0.4f; // max linear speed (m/s)

bool redraw = true;

bool on = false;
bool released = true;
bool changed = false;

float W1_;
float W2_;

differential_drive::Speed speed_msg;
differential_drive::Lights lights_msg;

ros::Subscriber key_sub;
ros::Subscriber enc_sub; //Subscribe to encoders to get velocity

ros::Publisher send_speed;
ros::Publisher send_lights;

struct KeyboardState
{
	KeyboardState()
		: up(false)
		, down(false)
		, left(false)
		, right(false)
	{

	}

	int8_t up;
	int8_t down;
	int8_t left;
	int8_t right;
};

//Callback function for the "/keyboard" topic. Keeps track of pressed keys and changes the motorspeed.
void receive_key(const KeyEvent::ConstPtr &msg)
{
	static KeyboardState keys;
    switch (msg->sym)
    {
        case SDLK_UP: keys.up = msg->pressed; break;
        case SDLK_DOWN: keys.down = msg->pressed; break;
        case SDLK_LEFT: keys.left = msg->pressed; break;
        case SDLK_RIGHT: keys.right = msg->pressed; break;
        case SDLK_l:
            if (msg->pressed)
            {
                if (released)
                {
                    on = !on;
                    changed = true;
                    released =false;
                }
            }
            else{
                released = true;
            }
            break;
	}

	// Calculate motor speeds
    float right = (keys.up-keys.down)*0.75 - (-keys.left+keys.right)*0.75; // * 0.5
    float left = (keys.up-keys.down)*0.75 + (-keys.left+keys.right)*0.75;  // * 0.5

	if (left < -1) left = -1;
	if (left > 1)  left = 1;
	if (right < -1) right = -1;
	if (right > 1)  right = 1;

    speed_msg.W2	= 1./r*V_max*left;
    speed_msg.W1	= 1./r*V_max*right;

    W1_ = speed_msg.W1;
    W2_ = speed_msg.W2;

    printf("left: %f right: %f\n", speed_msg.W2, speed_msg.W1);

    lights_msg.on = on;
    printf("Lights on: %u \n",on);
}

void encodersCallback(const Encoders::ConstPtr &msg)
{
    float w1_read = (msg->delta_encoder1)*(2.0f*M_PI/360)/(msg->timestamp/1000.0f);
    float w2_read = (msg->delta_encoder2)*(2.0f*M_PI/360)/(msg->timestamp/1000.0f);
    ROS_WARN("W1 read %.3f, W2 read: %.3f",w1_read,w2_read);
}

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "KeyboardControl");
	
	ros::NodeHandle n;

	key_sub = n.subscribe("/human/keyboard", 1000, receive_key); //when "/keyboard" topic is received, call back receive_key function
//    enc_sub = n.subscribe("/motion/Encoders", 1000, encodersCallback);

	send_speed = n.advertise<differential_drive::Speed>("/motion/Speed", 1000);
    send_lights = n.advertise<differential_drive::Lights>("/control/Lights", 1000);

	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		loop_rate.sleep(); // Sleep for 10 ms
//        uint32_t t =
        speed_msg.header.stamp.nsec = ros::Time::now().nsec;
        lights_msg.header.stamp.nsec = ros::Time::now().nsec;

		send_speed.publish(speed_msg);
//        if(changed)
//        {
//            changed = false;
            send_lights.publish(lights_msg);
//        }
		ros::spinOnce();   // Process ros messages	
	}

	return 0;
}


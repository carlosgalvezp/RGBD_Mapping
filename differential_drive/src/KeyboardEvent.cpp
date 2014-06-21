 /*
 * Human Interface.cpp
 * --------------------
 * Copyright : (c) 2013, Germain Haessig <germain.haessig@ens-cachan.fr>
 * Licence   : BSD3
 * part of the differential_drive ROS package, for ROBOT13@KTH
 * 
 * Enable keyboard control, in the oppened window
 */

#include "ros/ros.h"
#include <string.h>
#include <algorithm>

#include "differential_drive/KeyEvent.h"

#include <SDL/SDL.h>
#include <math.h>

using namespace differential_drive;
ros::Publisher key_pub;

#define WIDTH 100
#define HEIGHT 100
#define BPP 4
#define DEPTH 32

bool quit = false;
bool redraw = true;


int main(int argc, char **argv) {
	ros::init(argc, argv, "KeyboardEvent"); //Creates a node named "KeyboardEvent"
	ros::NodeHandle n;
	key_pub = n.advertise < KeyEvent > ("/human/keyboard", 100000);
	
	SDL_Surface *screen;
	SDL_Event event;

	if (SDL_Init(SDL_INIT_VIDEO) < 0)
		return 1;

	if (!(screen = SDL_SetVideoMode(WIDTH, HEIGHT, DEPTH, SDL_HWSURFACE))) {
		SDL_Quit();
		printf("SDL_SetVideoMode failed to open window\n");
		return 1;
	}

	SDL_WM_SetCaption("KeyboardEvent", "KeyboardEvent");

	ros::Rate loop_rate(30); // frequency
	while (ros::ok() && !quit) {
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
			case SDL_QUIT:
				quit = true;
				break;
			case SDL_KEYDOWN:
			case SDL_KEYUP:
			{
				printf("key '%s' %s\n", SDL_GetKeyName( event.key.keysym.sym ), event.type == SDL_KEYDOWN ? "PRESSED" : "RELEASED");
				// Send key event message
				timeval now;
				gettimeofday(&now, NULL);
				KeyEvent key;
				key.timestamp = now.tv_sec+double(now.tv_usec)/1000000.0;
				key.sym = event.key.keysym.sym;
				key.pressed = (event.type == SDL_KEYDOWN);
				key.name = SDL_GetKeyName( event.key.keysym.sym );
				key_pub.publish(key);
				break;
			}
			}
		}

		loop_rate.sleep(); // Sleep for 10 ms
		ros::spinOnce(); // Process ros messages
	}

	SDL_Quit();
	return EXIT_SUCCESS;
}


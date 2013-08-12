/*
 * keyboard.cpp
 *
 *  Created on: Dec 5, 2012
 *      Author: biorobotics
 */



#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <ros/ros.h>

#include "keyboard.h"
#include "world.h"
#include "main.h"
#include "transforms.h"
#include "fls_vision/PegNumLoc.h"

using namespace std;
struct termios cooked, raw;
int kfd = 0;

extern ros::NodeHandle * __rosnode;
extern const int armid_left;
extern const int armid_right;
int printthisthinghackhackhackhack = 0;

enum {
	ve_golf_course,
	ve_attractive,
	ve_jetstream
} static ve_mode = ve_attractive;

void resetkeys()
{
	tcsetattr(kfd, TCSANOW, &cooked);
}
void haptic_world::keyboardLoop()
{
    char c;
    assistive_feature* (*fp_createEnv)(int) = &create_zero_env;

    // setup peg_location_service
	ros::ServiceClient client = __rosnode->serviceClient<fls_vision::PegNumLoc>("PegNumLoc");
    ROS_INFO("Peg request service ready");

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    cout << "KEYS:\n";
  	cout << " Q: Quit \n";
  	cout << " 0: Clear VE \n";
  	cout << " L: Get peg locations\n";
  	cout << "VE Select:\n";
  	cout << " J: Jetstream env\n";
  	cout << " G: Golf course env\n";
  	cout << " A: Atractive cylinder env\n";
  	cout << "VE Control:\n";
  	cout << " N: Next phase\n";
  	cout << " P: Prev phase\n";

  	int phase=0;
    for(;;)
    {
        boost::this_thread::interruption_point();

        // get the next event from the keyboard
        int num;
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
            continue;
        }

        switch(c)
        {

        // clear VE
        case '0':
        {
        	features.clear_all_features();
        	ROS_INFO("features cleared");
    		break;
        }
        // print current location(s)
        case 'r':
        case 'R':
        {
        	for (unsigned int i=0;i<hips.size();i++)
        		ROS_INFO( "HIP[%d]: (%f,\t%f,\t%f)", i, hips[i]->x(), hips[i]->y(), hips[i]->z() );

        	printthisthinghackhackhackhack = 1;
        	break;
        }

        // use Jetstream VE
        case 'l':
        case 'L':
        {
        	btVector3 fakepegs[1] = {
        			btVector3(-0.375999,   0.178599,       0.178599)
        	};
        	fls_vision::PegNumLoc srv;
			srv.request.a = 1;

			int testing = 0;

			if (testing)
			{
				ROS_INFO("Number of test pegs: 5");
				int peg = 1;
				numPhases = peg*2;
				phase = 0;
				for (int pegcount = 0; pegcount < peg; pegcount++)
				{
					btTransform temp;
					temp.setOrigin( fakepegs[pegcount] );
					ROS_INFO("Bp: (x,y,z) (mm): (%f,\t%f,\t%f)", temp.getOrigin()[0], temp.getOrigin()[1] ,temp.getOrigin()[2] );

					temp = xform_ravenbase2ravenrcm( temp, armid_right ); // convert mm to meters
					ROS_INFO("Rp: (x,y,z) (mm): (%f,\t%f,\t%f)", temp.getOrigin()[0], temp.getOrigin()[1] ,temp.getOrigin()[2] );

					temp = xform_raven2ITP( temp, armid_right);
					ROS_INFO("Ip:(x,y,z) (mm): (%f,\t%f,\t%f)", temp.getOrigin()[0], temp.getOrigin()[1] ,temp.getOrigin()[2] );

					pegs[pegcount] = temp.getOrigin();
				}
			}
			else if ( client.call(srv))
			{
				ROS_INFO("Number of Pegs: %lld", srv.response.pegnum);
				int peg = srv.response.pegnum;
				numPhases = peg*2;
				phase = 0;

				for (int pegcount = 0; pegcount < peg; pegcount++)
				{
					btTransform temp;
					temp.setOrigin( btVector3(srv.response.x[pegcount] / 1000, srv.response.y[pegcount] / 1000, srv.response.z / 1000) );
					ROS_INFO("Bp: (x,y,z) (mm): (%f,\t%f,\t%f)", temp.getOrigin()[0], temp.getOrigin()[1] ,temp.getOrigin()[2] );

					temp = xform_ravenbase2ravenrcm( temp, armid_right ); // convert mm to meters
					ROS_INFO("Rp: (x,y,z) (mm): (%f,\t%f,\t%f)", temp.getOrigin()[0], temp.getOrigin()[1] ,temp.getOrigin()[2] );

					temp = xform_raven2ITP( temp, armid_right);
					ROS_INFO("Ip:(x,y,z) (mm): (%f,\t%f,\t%f)", temp.getOrigin()[0], temp.getOrigin()[1] ,temp.getOrigin()[2] );

					pegs[pegcount] = temp.getOrigin();
				}
			}

			else {
				ROS_ERROR("Failed to call service PegNumLoc");
			}
			break;
		}

        // use Jetstream VE
        case 'j':
        case 'J':
        {
        	ve_mode = ve_jetstream;
        	phase=0;
        	fp_createEnv = &create_jetstream_env;
        	features.clear_all_features();
        	features.add_feature( fp_createEnv(phase) );
        	ROS_INFO("New env: jetstream");
        	break;
        }

        // use golf courseVE
        case 'g':
        case 'G':
        {
        	ve_mode = ve_golf_course;
        	phase=0;
        	fp_createEnv = &create_golfcourse_env;
        	features.clear_all_features();
        	features.add_feature( fp_createEnv(phase) );
        	ROS_INFO("New env: golf_course");
        	break;
        }

        // use attractive cylinder mode
        case 'a':
        case 'A':
        {
        	ve_mode = ve_attractive;
        	phase=0;
        	fp_createEnv = &create_attractive_env;
        	features.clear_all_features();
        	features.add_feature( fp_createEnv(phase) );
        	ROS_INFO("New env: attractive cylinder");
        	break;
        }

        // quit
        case 'q':
        case 'Q':
        {
        	tcsetattr(kfd, TCSANOW, &cooked);
        	ros::shutdown();
        	break;
        }

        // Next phase
        case 'n':
        case 'N':
        	ROS_INFO("Next fixture");
        	phase++;
        	features.clear_all_features();
        	features.add_feature( fp_createEnv(phase) );
        	break;

		// Next phase
		case 'p':
		case 'P':
			ROS_INFO("Prev fixture");
        	phase = (phase > 0) ? phase-1 : 0;
        	features.clear_all_features();
        	features.add_feature( fp_createEnv(phase) );
			break;

        default:
        	ROS_INFO("Keypressed %c", c);
        	break;
        }

    }
}






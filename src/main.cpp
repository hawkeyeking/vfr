/** File: vfr.cpp
* Author: H. Hawkeye King
* Created 5-Aug-2011
*
*  Main entry for virtual fixture renderer.
*
*/

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <raven_2/raven_state.h>
#include <raven_2/raven_automove.h>
#include "main.h"
#include "world.h"
#include "fixture_types.h"
#include "network.h"
#include "keyboard.h"
#include "transforms.h"

unsigned int rlvl =0;;
unsigned long int gTime = 0;
ros::Publisher pub_automove;

const static int use_right = 1;
const static int use_left  = 0;
const static double raven_right_zoffset = 0.140;

extern char* master_addr;
haptic_world *hw;
HIP* leftarm;
HIP* rightarm;

// indices for addressing data in the ravenstate messages
int l_index, r_index;

ros::NodeHandle * __rosnode;

/**
 * msg receive callback
 */
static const double um2m = 1/(1000.0*1000.0);
void positionCallback(const raven_2::raven_state::ConstPtr& msg)
{
	gTime++;
	btTransform ravenpos[2];

	l_index = msg->type[0]==GOLD_SERIAL ? 0 : 1;
	r_index = 1-l_index;

	if (use_left)
	{
		ravenpos[0].setOrigin( 	btVector3(  msg->pos_d[3*l_index + 0] * um2m,
											msg->pos_d[3*l_index + 1] * um2m,
											msg->pos_d[3*l_index + 2] * um2m
											) );
		ravenpos[0] = xform_raven2ITP(ravenpos[0], armid_left);
		leftarm->set_pos_from_xf(ravenpos[0], msg->hdr.stamp);
	}

	if (use_right)
	{
		ravenpos[1].setOrigin( 	btVector3(  msg->pos_d[3*r_index + 0] * um2m,
											msg->pos_d[3*r_index + 1] * um2m,
											msg->pos_d[3*r_index + 2] * um2m
											) );
		ravenpos[1] = xform_raven2ITP(ravenpos[1], armid_right);
		rightarm->set_pos_from_xf(ravenpos[1], msg->hdr.stamp);
	}

	rlvl = msg->runlevel;
    if (rlvl > 2)
    {
    	hw->update_HIP_forces();
    }
    else
    {
    	hw->zero_HIP_forces();
    }

	static ros::Time tnow;
	static ros::Time tlast = tlast.now();
	static ros::Time thdr;
	static ros::Duration dt, dt2;

	// timing test
	tnow = tnow.now();
	thdr = msg->hdr.stamp;
	dt = tnow - thdr;
	dt2 = tnow - tlast;
//	if (dt2.toSec() >= 1 )
//	{
//	    ROS_INFO("lid: %d\t dt: \%3f\t tn: \%3f\t th: \%3f",msg->last_seq, dt.toSec(), tnow.toSec(), thdr.toSec());
//		tlast=tnow;
//	}

    // Send force to master
    send_data(  leftarm->get_force(),
    			rightarm->get_force(),
    			msg->last_seq );
}


/*
 * publish_automove()
 *     Move robot via ros command.
 *
 *     Header      hdr
 *     int32[6]    del_pos
 *
 */
void publish_automove(btVector3 in_pos_incr, int in_arm_id)
{
	int id = ( in_arm_id == armid_left ) ? l_index : r_index;

	btVector3 pos_incr( xform_ITP2Raven(in_pos_incr, in_arm_id) );

	static raven_2::raven_automove rauto;
	rauto.hdr.stamp = rauto.hdr.stamp.now();

	rauto.del_pos[id*3  ] = (int) (pos_incr[0] / um2m);
	rauto.del_pos[id*3+1] = (int) (pos_incr[1] / um2m);
	rauto.del_pos[id*3+2] = (int) (pos_incr[2] / um2m);
	rauto.del_pos[(1-id) *3  ] = 0;
	rauto.del_pos[(1-id) *3+1] = 0;
	rauto.del_pos[(1-id) *3+2] = 0;

	pub_automove.publish(rauto);
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "VFR");
    __rosnode = new ros::NodeHandle;
    ros::NodeHandle nh;
    std::string master_addr_param;
    if (nh.getParam("teleop_master_ip", master_addr_param))
    {
    	updateMasterAddr(master_addr_param);
    }

    hw = new haptic_world;

    // initialize haptic rendering environment
    if (use_left)
    	leftarm  = hw->addHIP(armid_left);
    else
    	leftarm = new HIP();

    if (use_right)
    	rightarm = hw->addHIP(armid_right);
    else
    	rightarm = new HIP();

    /**
    * Subscribe to ravenstate to get position informatin updates
     */
    ros::TransportHints TH;
    ros::Subscriber sub = __rosnode->subscribe("ravenstate", 1000, positionCallback, TH.udp() );
    ROS_INFO("Now listening for ravenstate");

	pub_automove = __rosnode->advertise<raven_2::raven_automove>("raven_automove", 1);
    ROS_INFO("Advertising raven_automove");

    ROS_INFO("Reading keyboard...");
    boost::thread t = boost::thread( boost::bind( &haptic_world::keyboardLoop, hw ));

    ros::spin();
    resetkeys();

    return 0;
}

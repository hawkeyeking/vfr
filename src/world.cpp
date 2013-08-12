/*
 * File: world.cpp
 * Author: H. Hawkeye King
 * Created October 2011
 *
 * Given a position, calculate force based on interaction with virtual environment model
 *
 */

/*
 *   render_force():
 *      Calculate device force.
 *      Return value: 0 on success, negative on failure.
 */

#include <ros/ros.h>
#include "world.h"
#include "main.h"

extern unsigned int rlvl;
extern unsigned long int gTime;
const static double peg_height = -0.17;
const static double block_height = 0.183;

const btVector3 peg_l_front( 0.0021, -0.023, 0.2024 );
const btVector3 peg_l_rear ( 0.1027, -0.023, 0.1962 );
double peg_incr = (peg_l_rear[0] - peg_l_front[0]) / 4 ;

btVector3 haptic_world::pegs[20]= {
		btVector3(0.045686,-0.081929,0.16966),
		btVector3(0.064275,-0.081677,0.17139),
		btVector3(0.080336,-0.081761,0.17413),
		btVector3(0.09829,-0.083797,0.17428),
		btVector3(0.11588,-0.08449,0.17593),
		btVector3(0.047441,-0.09977,0.17156),
		btVector3(0.063301,-0.099892,0.17288),
		btVector3(0.080916,-0.10121,0.1738),
		btVector3(0.098858,-0.10143,0.17529),
		btVector3(0.11763,-0.10236,0.1771),
		btVector3(0.046726,-0.11784,0.17263),
		btVector3(0.065629,-0.11714,0.17351),
		btVector3(0.081524,-0.11841,0.17499),
		btVector3(0.10003,-0.11823,0.17655),
		btVector3(0.11711,-0.11923,0.17827),
		btVector3(0.046562,-0.13513,0.17377),
		btVector3(0.065681,-0.13494,0.17505),
		btVector3(0.083019,-0.13532,0.17581),
		btVector3(0.10067,-0.13695,0.17827),
		btVector3(0.11697,-0.13741,0.17949)
};

btVector3 haptic_world::blocks[15] = {
		btVector3(-0.16671,0.047444,0.018618),
		btVector3(-0.16761,0.064495,0.019051),
		btVector3(-0.16881,0.081063,0.01999),
		btVector3(-0.16868,0.098449,0.019652),
		btVector3(-0.17253,0.11647,0.019726),
		btVector3(-0.16579,0.046983,0.036867),
		btVector3(-0.16682,0.06306,0.036741),
		btVector3(-0.16881,0.080837,0.036934),
		btVector3(-0.17084,0.098513,0.037503),
		btVector3(-0.17329,0.11573,0.03769),
		btVector3(-0.16711,0.046131,0.053597),
		btVector3(-0.16837,0.063377,0.053978),
		btVector3(-0.16966,0.080786,0.053979),
		btVector3(-0.17129,0.098256,0.054127),
		btVector3(-0.17358,0.1157,0.053583)
};

int haptic_world::numPhases = 10;

/***
 *
 * haptic worlds
 *
 * ***/
haptic_world::haptic_world()
{
//	horiz_plane *zp = new horiz_plane(peg_height, 700.0, 0.0);
//	features.add_feature( (assistive_feature*)zp );
}

HIP* haptic_world::addHIP(int hipid)
{
	hips.push_back( new HIP(hipid) );
	ROS_INFO("added hip with hipid: %d", hips.back()->id() );
	return hips.back();
}

void haptic_world::update_HIP_forces()
{
	//	ROS_INFO("numfeatures:%d", features.size());
	for(unsigned int hit = 0; hit < hips.size(); hit++)
	{
		double force[3] = {0,0,0};
		features.calculate_effect( *hips[hit], force);
		hips[hit]->set_force(force);
	}
}

const double zero_force[3]={0,0,0};
void haptic_world::zero_HIP_forces()
{
	for(unsigned int hit = 0; hit < hips.size(); hit++)
	{
		hips[hit]->set_force(zero_force);
	}
}

assistive_feature* haptic_world::create_zero_env(int phase)
{
	feature_list *fl = new feature_list(assistive_feature::af_min_force_intersect);
	horiz_plane *zp = new horiz_plane(0.12, 700.0, 0.0);
	fl->add_feature( (assistive_feature*)zp );
	return (assistive_feature*) fl;
}


assistive_feature* haptic_world::create_jetstream_env(int phase)
{
	int trgnum = (phase % numPhases) / 2;

	btVector3 targ;
	if (phase % 2 == 1)
	{
		ROS_INFO("PEG phase \tp: %d \ttp:%d \t@:  (%f,\t%f,\t%f)",phase, trgnum, pegs[trgnum][0], pegs[trgnum][1] ,pegs[trgnum][2] );
		targ = pegs[ trgnum ];
	}
	else
	{
		ROS_INFO("BLO phase \tp: %d \ttp:%d \t@:  (%f,\t%f,\t%f)",phase, trgnum, pegs[trgnum][0], pegs[trgnum][1] ,pegs[trgnum][2] );
		targ = pegs[ trgnum ];
	}

	jetstream *js = new jetstream(targ, 0.01);
	return (assistive_feature*)js;
}

assistive_feature* haptic_world::create_attractive_env(int phase)
{
	int trgnum = (phase % numPhases) / 2;

	btVector3 targ;
	if (phase % 2 == 1)
	{
		ROS_INFO("PEG phase \tp: %d \ttp:%d \t@:  (%f,\t%f,\t%f)",phase, trgnum, pegs[trgnum][0], pegs[trgnum][1] ,pegs[trgnum][2] );
		targ = pegs[ trgnum ];
	}
	else
	{
		ROS_INFO("BLO phase \tp: %d \ttp:%d \t@:  (%f,\t%f,\t%f)",phase, trgnum, pegs[trgnum][0], pegs[trgnum][1] ,pegs[trgnum][2] );
		targ = blocks[ trgnum ];
	}
	vert_line *vl = new vert_line(0.01, targ[0], targ[1] ,4.0, 0.4);
	return (assistive_feature*) vl;
}


assistive_feature* haptic_world::create_golfcourse_env(int phase)
{
	const static double cylsize = 0.006;
	const static double envstiffness = 200.0;

	feature_list *fl = new feature_list(assistive_feature::af_min_force_intersect);
	fl->add_feature( (assistive_feature*)new horiz_plane(peg_height, envstiffness, 0.0) );

	int trgnum = (phase % numPhases) / 2;

	btVector3 targ;
	if (phase % 2 == 1)
	{
		ROS_INFO("PEG phase \tp: %d \ttp:%d \t@:  (%f,\t%f,\t%f)",phase, trgnum, pegs[trgnum][0], pegs[trgnum][1] ,pegs[trgnum][2] );
		targ = pegs[ trgnum ] - btVector3(0,0,0.08);
	}
	else
	{
		ROS_INFO("BLO phase \tp: %d \ttp:%d \t@:  (%f,\t%f,\t%f)",phase, trgnum, pegs[trgnum][0], pegs[trgnum][1] ,pegs[trgnum][2] );
		targ = blocks[ trgnum ] - btVector3(0,0,0.08);
	}

	vert_cylinder *zc;

	fl->add_feature( (assistive_feature*)new vert_cylinder(cylsize, targ[0], targ[1], envstiffness, 0.0) );

	return (assistive_feature*) fl;
}

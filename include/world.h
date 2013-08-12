/*
 * File: world.h
 * Author: H. Hawkeye King
 * Created October 2011
 *
 * Given a position, calculate force based on interaction with virtual environment model
 *
 */

#include <vector>
#include <limits>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "fixture_types.h"

#ifndef RENDER_FORCE_H_
#define RENDER_FORCE_H_

/* CLASS: haptic_world
 *   Represents the haptic rendering world
 */
class haptic_world
{

public:
	haptic_world();

	static btVector3 pegs[20];
	static btVector3 blocks[15];

	HIP* addHIP(int);
	void update_HIP_forces();
	void zero_HIP_forces();

	static assistive_feature* create_zero_env(int phase=0);
	static assistive_feature* create_jetstream_env(int phase=0);
	static assistive_feature* create_attractive_env(int phase=0);
	static assistive_feature* create_golfcourse_env(int phase=0);

	void keyboardLoop();

private:
	// TODO: make this a vector of positions
	std::vector<HIP*> hips;
	feature_list features;
	static int numPhases;
};


#endif /* RENDER_FORCE_H_ */


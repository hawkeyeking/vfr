/*
 * File: feature_types.cpp
 * Author: H. Hawkeye King
 * Created March 2013
 *
 * Implements virtual fixture primitives and stuff
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
extern int printthisthinghackhackhackhack;

/***
 *
 *
 * haptic interaction points
 *
 * ***/
int HIP::hip_idnumbers;

HIP::HIP()
{
	hip_id = hip_idnumbers;
	hip_idnumbers++;
	hip_t=hip_tlast = 0;
	for (int i=0;i<3;i++)
	{
		hip_time_hist[i] = ros::Time::now();
		for (int j=0; j<hip_historysize; j++)
		{
			hip_pos_hist[i][j] = 0;;
		}
		hip_pos[i]=0;
		hip_force[i]=0;
	}
}


HIP::HIP(int in_hipid)
{
	hip_id = in_hipid;
	hip_idnumbers++;
	hip_t = hip_tlast = 0;
	for (int i=0;i<3;i++)
	{
		hip_time_hist[i] = ros::Time::now();
		for (int j=0; j<hip_historysize; j++)
		{
			hip_pos_hist[i][j] = 0;;
		}
		hip_pos[i]=0;
		hip_force[i]=0;
	}
}

void HIP::set_pos(double in_pos[3], const ros::Time& in_t)
{
	for (int i=0;i<3;i++)
	{
		set_pos_at(i, in_pos[i], in_t);
	}
}

void HIP::set_pos_from_xf(btTransform &in_xf, const ros::Time& in_t)
{
	static int count;
	count++;
	btVector3 in_pos = in_xf.getOrigin();
	for (int i=0;i<3;i++)
	{
		set_pos_at(i, in_pos[i], in_t);
	}
}

void HIP::set_pos_at(int idx, double in_pos, const ros::Time& in_t)
{
	// check for multiple calls in one timestep
	if (hip_time_hist[hip_t] != in_t)
	{
		hip_tlast = hip_t;
		hip_t = (hip_t >= hip_historysize-1) ? 0 : hip_t+1;

		hip_time_hist[hip_t] = in_t;
	}

	hip_pos[idx] = in_pos;
	hip_pos_hist[idx][hip_t]  = in_pos;

	hip_vel[idx] = 0;
	for (int j=1;j<=hip_historysize; j++)
	{
		hip_vel[idx] += (hip_pos_hist[idx][j%hip_historysize] - hip_pos_hist[idx][j-1]) / (hip_time_hist[j%hip_historysize].toSec() - hip_time_hist[j-1].toSec() );
	}
	hip_vel[idx] = hip_vel[idx]/hip_historysize;

//	if (idx==0 && hip_id ==0 && rlvl >2)
////		ROS_INFO("id: %d , idx: %d , hipt / last: %d \t/  %d", hip_id, idx,  hip_t, hip_tlast);
//		ROS_INFO("vel0 (%d/%d): %0.3f", hip_t, hip_historysize, hip_vel[idx]);

}

void HIP::set_force(const double *in_force)
{
	for (int i=0;i<3;i++)
		hip_force[i] = in_force[i];
}

// returns number of seconds in previous update cycle.  Will probably be in the millisecond range
double HIP::get_last_time_incr()
{
	return hip_time_hist[hip_t].toSec() - hip_time_hist[hip_tlast].toSec();
}


/***
 *
 * Assistive features
 *    jestream--- auto move with velocity proportional to height
 *
 * ***/
bool jetstream::calculate_effect(HIP& hip, double* out_force)
{
	// 1 = 1cm/sec at a depth of 1cm.  == 1m/sec at 1m
	const static double velocity_scale = 2; // units 1 (cm/sec)/cm = 1/sec

	btVector3 pos_incr;

	for (int i=0;i<3;i++)
		out_force[i]=0;

	bool contact = is_in_contact(hip);
	if ( contact && distance_to_target(hip) > target_radius)
	{
		// Calculate desired velocity
		// Velocity command is a function of penetration depth into the feature
		btVector3 velocity = normalized_direction_to_target(hip) * velocity_scale * sqrt(penetration_depth(hip)); // units: m/sec

		// Calculate a position increment1
		if (hip.get_last_time_incr() < TENTH_SECOND)
		{
			pos_incr = hip.get_last_time_incr() * velocity;
		}

		// Send position increment to robot
		publish_automove(pos_incr, hip.id());
	}

	return contact;
}

bool jetstream::is_in_contact(HIP& hip)
{
	return ( penetration_depth(hip) != 0 );
}


double jetstream::penetration_depth(HIP& hip)
{

	// HACK HACK HACK
	// HACK HACK HACK
	// HACK HACK HACK
	const static double jetstream_height = 0.17;

	double depth = (jetstream_height - hip[2]);

	if (depth > 0)
		return depth;

	return 0.0;
}

btVector3 jetstream::normalized_direction_to_target(HIP& hip)
{
	// direction = target - position
	btVector3 dir = target_pos - hip.get_pos();


	if (printthisthinghackhackhackhack == 1)
	{
		ROS_INFO("peg: (%f,\t%f,\t%f)", target_pos[0],target_pos[1],target_pos[2] );
		ROS_INFO("hip: (%f,\t%f,\t%f)", hip.get_pos()[0],hip.get_pos()[1],hip.get_pos()[2] );
		ROS_INFO("dir: (%f,\t%f,\t%f)", dir[0],dir[1],dir[2] );
		printthisthinghackhackhackhack = 0;
	}
	dir[2]=0;
	dir.normalize();
	return dir;
}

double jetstream::distance_to_target(HIP& hip)
{
	// direction = target - position
	return (double) ( target_pos - hip.get_pos() ).length();
}






/***
 *
 * Auto-move
 *
 *
 ***/
const static int maxvel = 1;
bool auto_move::calculate_effect(HIP& hip, double* out_force)
{

	// 1cm/sec at a depth of 1cm.  == 1m/sec at 1m
	const static double velocity_scale = 1; // units 1 (cm/sec)/cm = 1/sec
	bool contact = false;
	btVector3 pos_incr;

	for (int i=0;i<3;i++)
		out_force[i]=0;

	// Calculate desired velocity
	// Velocity command is a function of penetration depth into the feature
	btVector3 velocity = normalized_direction_to_target(hip) * velocity_scale * penetration_depth(hip); // units: m/sec


	// Calculate a position increment1
	// pos_incr = timestep * vel_vector = t * d/t
	if (hip.get_last_time_incr() < TENTH_SECOND)
	{
		pos_incr = hip.get_last_time_incr() * velocity;
	}

	// Send position increment to robot
	publish_automove(pos_incr, hip.id());

	return contact;
}




/***
 *
 *
 * Haptic plane with constant z
 *
 *
 * ***/
horiz_plane::~horiz_plane()
{
	return;
}

bool horiz_plane::calculate_effect(HIP &hip, double * out_force)
{
	bool contact = false;
	for (int i=0;i<3;i++) out_force[i]=0;

	if (hip.z() < elevation_coord)
	{
		contact = true;
		double depth = elevation_coord - hip.z();
		out_force[2] = depth * kp - hip.zvel() * kd;
	}

	return contact;
}


/***
 *
 *
 * Haptic cylinder aligned to Z-axis
 *
 *
 * ***/
vert_cylinder::~vert_cylinder()
{
	return;
}

bool vert_cylinder::calculate_effect(HIP &hip, double * out_force)
{
	bool contact = false;
	for (int i=0;i<3;i++) out_force[i]=0;

	// distance from radius of cylinder
	double dist = sqrt( pow(y_coord - hip.y(), 2) + pow(x_coord - hip.x(), 2)) - radius;

	if (dist > 0)
	{
		contact = true;

		double fp   = kp * dist;
		double vel  = sqrt( pow(hip.yvel(), 2) + pow(hip.xvel(), 2));
		double fv   = kd * vel;
		out_force[0] = fp * (x_coord - hip.x()) / (dist + radius) ;//    -    kd * fv * hip.xvel() / vel;
		out_force[1] = fp * (y_coord - hip.y()) / (dist + radius) ;//    -    kd * fv * hip.yvel() / vel;
	}

	return contact;
}






/***
 *
 *
 * Line of attraction parallel to Z-axis
 *
 *
 * ***/
vert_line::~vert_line()
{
	return;
}

bool vert_line::calculate_effect(HIP &hip, double * out_force)
{
	bool contact = true;
	for (int i=0;i<3;i++) out_force[i]=0;

	double dist = sqrt( pow(y_coord - hip.y(), 2) + pow(x_coord - hip.x(), 2));

	long double fmag = foam_magnet(dist);
	out_force[0] = fmag * (x_coord - hip.x()) / (dist);
	out_force[1] = fmag * (y_coord - hip.y()) / (dist);

	ROS_INFO("force:%f\t%f\t%f", out_force[0],out_force[1],out_force[2]);

	return contact;
}

long double vert_line::foam_magnet(long double in_dist)
{
	long double dist = (in_dist - radius/2) >0 ?  (in_dist - radius/2) : 0;
	long double rad = radius * 2;
	const static long double ee = 2.71828;
	long double b = 3.0;
	long double a = rad/b;
	long double x = dist;
	long double out_force =  peak_force    *    powl((ee/(b*a)), b)   *    powl(x,b)   *   powl(ee,-x/a);

	if ((dist/2) < 2*rad)
		out_force = peak_force * (1 - cosl(M_PI*((dist/2)/rad))); //700 * dist;

	else
		out_force = 0;

//	ROS_INFO("fm: %0.3Lf = %Lf \t * %Lf /t* %Lf ", out_force, powl((ee/(b*a)), b)  ,    powl(x,b)  ,   powl(ee,-x/a) );

	return (double)out_force;
}






/***
 *
 *
 * Haptic prism aligned to Z-axis
 *
 *
 * ***/
vert_rect::~vert_rect()
{
	return;
}

bool vert_rect::calculate_effect(HIP &hip, double * out_force)
{
	bool contact = false;
	for (int i=0;i<3;i++) out_force[i]=0;

	if (hip.get_id() != 0)
		return false;

	if ( hip.x()-x_coord > radius )
	{
		contact=true;
		out_force[0] = ((x_coord+radius) - hip.x()) * kp - hip.xvel() * kd;
	}
	else if ( hip.x()-x_coord <  -radius )
	{
		contact=true;
		out_force[0] = ((x_coord-radius) - hip.x()) * kp - hip.xvel() * kd;
	}


	if ( hip.y()-y_coord > radius )
	{
		contact=true;
		out_force[1] = ((y_coord+radius) - hip.y()) * kp - hip.yvel() * kd;
	}
	else if ( hip.y()-y_coord <  -radius )
	{
		contact=true;
		out_force[1] = ((y_coord-radius) - hip.y()) * kp - hip.yvel() * kd;
	}

	return contact;
}



/***
 *
 *
 * feature lists
 *
 *
 ***/
bool feature_list::calculate_effect(HIP &hip, double *out_force)
{
	bool contact = false;
	double min_force_mag = 65536;

	for (int i=0; i<3; i++)
		out_force[i] = 0;

	for(unsigned int fidx=0; fidx < vfs.size(); fidx++)
	{
		double vf_force[3] = {0,0,0};
		double f_mag;

		bool feature_contact = vfs[fidx]->calculate_effect(hip, vf_force);
		f_mag = vf_force[0]*vf_force[0] + vf_force[1]*vf_force[1] + vf_force[2]*vf_force[2];

		switch(af_combo)
		{
		case af_additive:
			for (int i=0; i<3; i++)
				out_force[i] += vf_force[i];
			contact |= feature_contact;
			break;

		case af_min_force_intersect:
			if (!feature_contact)
			{
				min_force_mag = -1;
				for (int i=0;i<3;i++)
					out_force[i] = 0;
				return false;
			}
			else if ( f_mag < min_force_mag )
			{
				min_force_mag = f_mag;
				for (int i=0;i<3;i++)
					out_force[i] = vf_force[i];
				contact = true;
			}
			break;

		default:
			break;
		}
	}
	return contact;
}

void feature_list::add_feature(assistive_feature* in_af)
{
	vfs.push_back(in_af);
}

void feature_list::clear_all_features()
{
	for(unsigned int fidx=0; fidx < vfs.size() ; fidx++)
		delete(vfs[fidx]);

	vfs.clear();
	listsize=0;
}

feature_list::~feature_list()
{
	clear_all_features();
}


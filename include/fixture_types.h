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

#ifndef FIXTURE_TYPES_H_
#define FIXTURE_TYPES_H_

const static double TENTH_SECOND = 0.100;

/* CLASS HIP - Haptic Interaction Points
 *    Represents the points of point-contact
 */

// TODO: use btTransform instead of double[3];
class HIP
{
private:
	btVector3 hip_pos;
	btVector3 hip_vel;
	btVector3 hip_force;
	int hip_id;
	static int hip_idnumbers;

	const static int hip_historysize = 10;
	double hip_pos_hist[3][hip_historysize];
	ros::Time hip_time_hist[hip_historysize];
	int hip_t, hip_tlast;

public:
	HIP();
	HIP(int);

	double get_last_time_incr();

	double x(){ return hip_pos[0]; }
	double y(){ return hip_pos[1]; }
	double z(){ return hip_pos[2]; }
	double xvel(){ return hip_vel[0]; }
	double yvel(){ return hip_vel[1]; }
	double zvel(){ return hip_vel[2]; }
	btVector3 vel(){ return hip_vel; }
	double fx(){ return hip_force[0]; }
	double fy(){ return hip_force[1]; }
	double fz(){ return hip_force[2]; }
	int id()  { return hip_id; }


	int get_id() { return hip_id; }
	btVector3 get_force() { return hip_force; }

	void set_pos(double in_pos[3], const ros::Time&);
	void set_pos_from_xf(btTransform &in_xf, const ros::Time& in_t);

	btVector3 get_pos() {return hip_pos;};

	void set_pos_at(int idx, double in_pos, const ros::Time& );
	void set_force(const double * in_force);

	HIP& operator = (double _setto[3])
	{
		for (int i=0; i<3; i++) hip_pos[i]=_setto[i];
		return *this;
	}

	double operator [](int i)
	{
		if (i>3)  return  std::numeric_limits<double>::signaling_NaN() ;
		else      return hip_pos[i];
	}

};





/* CLASS: assistive feature
 * represents a "thing" or a "field" or other element of a virtual world
 */
class assistive_feature
{
public:
	enum af_combination_rules
	{
		af_nocombo = 0,
		af_additive = 1,
		af_min_force = 2,
		af_min_force_intersect=3,
		af_combination_rules_last=4
	};

	enum af_types
	{
		af_notype = 0,
		af_list = 1,
		af_horiz_plane = 2,
		af_vert_cyl = 3,
		af_vert_line = 4,
		af_jetstream = 5,
		af_types_last=6
	};

public:
	assistive_feature() {af_type = af_notype;}
	assistive_feature(af_types in_type) :
		af_combo(af_nocombo),
		af_type(in_type)
	{}
	assistive_feature(af_types in_type, af_combination_rules in_combo) :
		af_combo(in_combo),
		af_type(in_type)
	{}

	virtual bool calculate_effect(HIP&, double*) = 0;
	virtual ~assistive_feature() { };

protected:
	af_combination_rules af_combo;
	af_types af_type;

};






/*
Class: jetstream

    Implements semi-autonomous feature for moving the robot towards a specified target.
    Target is assumed to be a ring with center and radius given by "target_pos" and "target_radius"
*/
class jetstream : assistive_feature
{
public:
	jetstream() :
		assistive_feature(af_jetstream, af_additive),
		target_pos(btVector3(0,0,0)),
		target_radius(0.5)
	{};

	jetstream(btVector3 in_pos, double in_radius) :
		assistive_feature(af_jetstream, af_additive),
		target_pos(in_pos),
		target_radius(in_radius)
	{
		return;
	};
	bool calculate_effect(HIP&, double*);
	bool is_in_contact(HIP&);
	double penetration_depth(HIP&);
	double distance_to_target(HIP& hip);
	btVector3 normalized_direction_to_target(HIP& hip);
	virtual ~jetstream() { };
	btVector3 get_target_pos() {return target_pos;};

private:
	btVector3 target_pos;
	double target_radius;

};





/*
Class: auto_move

    Implements semi-autonomous feature for moving the robot around

*/
class auto_move : assistive_feature
{
public:
	auto_move() :
		assistive_feature(af_jetstream, af_additive),
		target_pos(btVector3(0,0,0)),
		target_radius(0.5)
	{};

	auto_move(btVector3 in_pos, double in_radius) :
		assistive_feature(af_jetstream, af_additive),
		target_pos(in_pos),
		target_radius(in_radius)
	{
		return;
	};
	bool calculate_effect(HIP&, double*);
	bool is_in_contact(HIP&);
	double penetration_depth(HIP&)      	{ return 0.0; };
	double distance_to_target(HIP& hip) 	{ return 0.0; };
	btVector3 normalized_direction_to_target(HIP& hip) { return btVector3(); };
	virtual ~auto_move() {}

private:
	btVector3 target_pos;
	double target_radius;

};






/*
 * CLASS: x_line
 *     Represents a shape or something that has a haptic effect.
 *     In this case a vertical line with an attracting force
 *
 */
class vert_line: assistive_feature
{
public:
	vert_line(	double in_radius,
				double in_x_coord,
				double in_y_coord,
				double in_peak_force, double in_damping) :
					assistive_feature(af_vert_line),
					radius(in_radius),
					x_coord(in_x_coord),
					y_coord(in_y_coord),
					peak_force(in_peak_force),
					damping(in_damping)
	{
		return;
	} ;

	bool calculate_effect(HIP&, double*);
	long double foam_magnet(long double dist);

	virtual ~vert_line();

private:
	double radius;
	double x_coord;
	double y_coord;

	double peak_force;
	double damping;
};




/*
 * CLASS: feature_list
 *   Represents a list of features or sub-features in a world
 *
 */
class feature_list : assistive_feature
{
public:
	feature_list() : assistive_feature(af_list,af_additive) {};
	feature_list(af_combination_rules combo) : assistive_feature(af_list,combo) {};
	bool calculate_effect(HIP&, double*); // returns true if in contact
	void add_feature(assistive_feature*);
	void clear_all_features();
	virtual ~feature_list();

	int size(){return vfs.size();};

private:
	std::vector<assistive_feature*> vfs;
	int listsize;

};



/*
 * CLASS: horiz_plane
 *     Represents a shape or something that has a haptic effect.
 *     In this case the plane Z = const
 *
 */
class horiz_plane: assistive_feature
{
public:
	horiz_plane(double in_elev_coord, double in_kp, double in_kd) :
		assistive_feature(af_horiz_plane),
		elevation_coord(in_elev_coord),
		kp(in_kp),
		kd(in_kd)
	{
		return;
	} ;

	bool calculate_effect(HIP&, double*);

	virtual ~horiz_plane();

private:
	double elevation_coord;
	double kp;
	double kd;
};


/*
 * CLASS: Z_cylinder
 *     Represents a shape or something that has a haptic effect.
 *     In this case the cylinder with z-axis aligned center
 *
 */
class vert_cylinder: assistive_feature
{
public:
	vert_cylinder(double in_radius,
			double in_x_coord,
			double in_y_coord,
			double in_kp, double in_kd) :
				assistive_feature(af_vert_cyl),
				radius(in_radius),
				x_coord(in_x_coord),
				y_coord(in_y_coord),
				kp(in_kp),
				kd(in_kd)
	{
		return;
	} ;

	bool calculate_effect(HIP&, double*);

	virtual ~vert_cylinder();

private:
	double radius;
	double x_coord;
	double y_coord;
	double kp;
	double kd;
};


class vert_rect: assistive_feature
{
public:
	vert_rect(double in_radius,
			double in_x_coord,
			double in_y_coord,
			double in_kp, double in_kd) :
				assistive_feature(af_vert_cyl),
				radius(in_radius),
				x_coord(in_x_coord),
				y_coord(in_y_coord),
				kp(in_kp),
				kd(in_kd)
	{
		return;
	} ;

	bool calculate_effect(HIP&, double*);

	virtual ~vert_rect();

private:
	double radius;
	double x_coord;
	double y_coord;
	double z_coord;
	double kp;
	double kd;
};


#endif

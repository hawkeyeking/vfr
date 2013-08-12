/*
 * transforms.h
 *
 *  Created on: Jul 2, 2013
 *      Author: biorobotics
 */

#include <tf/transform_datatypes.h>



const static btTransform ravenbase2ravenrcm_L(btMatrix3x3(0, 0, 1,   0,-1, 0,  1,0,0),  btVector3(0.007, 0.061, -0.30071));
const static btTransform ravenbase2ravenrcm_R(btMatrix3x3(0, 0, 1,  0,1, 0,  -1,0,0),  btVector3(-0.007, -0.061, -0.30071) );

const static btTransform raven_to_mantis_L( btMatrix3x3(0, 1,0,  0,0, 1,  1,0,0), btVector3(0,0,0));
const static btTransform raven_to_mantis_R( btMatrix3x3(0,-1,0,  0,0,-1,  1,0,0), btVector3(0,0.153, 0));
//const static double raven_r_z_offset_to_l = -0.153;

const static btTransform raven_to_ITP_L(btMatrix3x3(0, -1,0,  0,0, 1,  -1,0,0), btVector3(0,0,0));
const static btTransform raven_to_ITP_R(btMatrix3x3(0, 1,0,  0,0, -1,  -1,0,0), btVector3(0,0,0));

const static btTransform mantis_to_ITP(btMatrix3x3(-1,0,0,  0,1,0,  -0,0,-1), btVector3(0,0,0));


btTransform xform_ravenbase2ravenrcm( btTransform &in_xf, int arm );
btTransform xform_raven2mantis(btTransform&, int arm );
btTransform xform_raven2ITP   (btTransform&, int arm );
btTransform xform_ITP2Raven   (btTransform&, int);
btTransform xform_ITP2Mantis  (btTransform&);


btVector3 xform_raven2mantis( btVector3, int arm );
btVector3 xform_raven2ITP(    btVector3, int arm );
btVector3 xform_ITP2Raven(    btVector3, int);
btVector3 xform_ITP2Mantis(   btVector3);

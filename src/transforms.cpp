/*
 * transforms.cpp
 *
 *  Created on: Jul 2, 2013
 *      Author: biorobotics
 */

#include "transforms.h"
#include "main.h"


btTransform xform_raven2mantis( btTransform &in_xf, int arm )
{
	if (arm == armid_left)  return(raven_to_mantis_L * in_xf);
	else                    return(raven_to_mantis_R * in_xf);
}

btTransform xform_ravenbase2ravenrcm( btTransform &in_xf, int arm )
{
	if (arm == armid_left)   return( ravenbase2ravenrcm_L * in_xf );
	else                     return( ravenbase2ravenrcm_R * in_xf );
}

btTransform xform_raven2ITP( btTransform &in_xf, int arm )
{
	if (arm == armid_left)   return( raven_to_ITP_L * in_xf);
	else                     return( raven_to_ITP_R * in_xf);
}

btTransform xform_ITP2Raven(btTransform &in_xf, int id)
{
	if ( id == armid_left)   return raven_to_ITP_L * in_xf;
	else                     return raven_to_ITP_R * in_xf;
}

btTransform xform_ITP2Mantis(btTransform &in_xf)
{
	return mantis_to_ITP.inverse() * in_xf;
}

btVector3 xform_raven2mantis( btVector3 &vec, int arm )
{
	if (arm == armid_left) return raven_to_mantis_L(vec);
	else                   return raven_to_mantis_R(vec);
}

btVector3 xform_raven2ITP( btVector3 &vec, int armid )
{
	if (armid == armid_left) 	return raven_to_ITP_L(vec);
	else                    return raven_to_ITP_R(vec);
}

btVector3 xform_ITP2Raven(btVector3 vec, int armid)
{
	if ( armid == armid_left) return (raven_to_ITP_L.inverse())(vec);
	else                   return (raven_to_ITP_R.inverse())(vec);
}

btVector3 xform_ITP2Mantis(btVector3 vec)
{
		return (mantis_to_ITP.inverse())(vec);
}



/*
 * network.h
 *
 *  Created on: Oct 25, 2012
 *      Author: biorobotics
 */

#ifndef NETWORK_H_
#define NETWORK_H_

#include <tf/transform_datatypes.h>


int send_data(const btVector3 &in_fleft, const btVector3 &in_fright, int);
int updateMasterAddr(std::string in_master_addr);

#endif /* NETWORK_H_ */

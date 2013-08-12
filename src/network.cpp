/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/**********************************************
 *
 * file: network.cpp
 * author: Hawkeye King
 * create date: 10/2012
 *
 * Send the daters!
 *
 *********************************************/

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <itp_teleoperation.h>

#include "main.h"

int sock=-1;
struct sockaddr_in server_addr;

std::string master_addr = "128.95.30.156";
const int master_port = 36001;

int initsock()
{
  struct hostent *host;
  host = (struct hostent *) gethostbyname(master_addr.c_str());

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(master_port);
  server_addr.sin_addr = *((struct in_addr *) host->h_addr);

  if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    perror("socket");
    return -1;
  }

  return sock;
}

int send_data(const btVector3& in_fleft, const btVector3& in_fright, int last_seq)
{
  if (sock<0)
    initsock();

  static v_struct vs;

  vs.last_sequence = last_seq;

  vs.fx[0] = (float)in_fleft[0] * 1000;
  vs.fy[0] = (float)in_fleft[1] * 1000;
  vs.fz[0] = (float)in_fleft[2] * 1000;

  vs.fx[1] = (float)in_fright[0] * 1000;
  vs.fy[1] = (float)in_fright[1] * 1000;
  vs.fz[1] = (float)in_fright[2] * 1000;

  bzero(&(server_addr.sin_zero), 8);

  sendto(sock, &vs, sizeof(vs), 0,
	 (struct sockaddr *) &server_addr, sizeof(struct sockaddr));

  return 0;

}

int updateMasterAddr(std::string in_master_addr)
{
  struct hostent *host;
  host = (struct hostent *) gethostbyname(in_master_addr.c_str());
  if (host == NULL)
    {
      ROS_ERROR("Host lookup failed.");
      return -1;
    }
  return 0;
}

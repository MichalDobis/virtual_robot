/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Norwegian University of Science and Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Norwegian University of Science and
 *     Technology, nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef VIRTUAL_ROBOT_PLUGIN_H_
#define VIRTUAL_ROBOT_PLUGIN_H_

#include <pho_robot_loader/robot_base.h>
#include <tf/transform_datatypes.h>

// Socket
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

//log
#include <iostream>
#include <fstream>

namespace pho {

class VIRTUAL_ROBOT : public pho_robot_loader::RobotBase
{
public:
  VIRTUAL_ROBOT();
  ~VIRTUAL_ROBOT();

  // Overloaded Virtual Functions
  bool initialize(std::string robot_ip, int port);
  bool connectOrAccept();
  int receiveRequestFromRobot();
  int sendMsgHeader(int message_type, int header_data);
  int sendTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory, int traj_number, bool is_fine);
  int sendData(int type, int number, std::vector<int> data);

  void stateListenerConnect(std::string robot_ip, int port);
  void stateListenerPublish();

private:

  const int NUM_OF_JOINTS = 6;
  const int MESSAGE_HEADER_SIZE = 32;
  const int MESSAGE_SUB_HEADER_SIZE = 32;
  const int MESSAGE_INIT_SIZE = 32;
  const int MESSAGE_ACK_SIZE = 32;
  const int MESSAGE_IK_SIZE = 32;
  const int MAX_TRAJ_SIZE = 100;
  const int MESSAGE_FROM_ROBOT_MAX_SIZE = 52;
  const int MAX_BUFFER_SIZE = 512;
  const int STATE_LISTENER_BUFFER_SIZE = 48;

  bool pc_server_;
  int sock_handle_;
  int sockfd_server_;
  struct sockaddr_in server_addr_;

  // Private functions
  std::vector <uint8_t> createMsgHeader(int operation_type, int operation_number, int size);

 void create_log(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory);

 uint8_t getRequest();

  // State listener member
  int state_listener_client_;
  geometry_msgs::Pose tool_state_;
  sensor_msgs::JointState robot_state_;

  bool initialized_moveit;

  std::ofstream outfile_diferencial;
  int number_of_fails;
  int attempt;

  double cycle_time_;
  double moving_robot_time_;

 //int cycle_iterator_;



};  // class VIRTUAL_ROBOT
}   // namespace

#endif  // VIRTUAL_ROBOT_PLUGIN_H

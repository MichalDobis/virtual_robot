/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Photoneo s.r.o.
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
 *   * Neither the name of the Photoneo nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
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

#include <pluginlib/class_list_macros.h>
#include <pho_robot_loader/robot_base.h>
#include <photoneo_virtual_robot_module/virtual_robot_plugin.h>

namespace pho {

VIRTUAL_ROBOT::VIRTUAL_ROBOT() : pho_robot_loader::RobotBase()
{

}

VIRTUAL_ROBOT::~VIRTUAL_ROBOT()
{

}

bool VIRTUAL_ROBOT::initialize(std::string robot_ip, int port)
{


  return true;
}

bool VIRTUAL_ROBOT::connectOrAccept()
{

  return true;
}

int VIRTUAL_ROBOT::receiveRequestFromRobot()
{
  static int request_counter = -1;
  int request;

    if (request_counter > 1002)
        ros::shutdown();

  switch (request_counter){
    case -1:
      request = pho_robot_loader::REQUEST::CUSTOMER_REQ;
          customer_data_.resize(2);
          customer_data_[0] = -1.24;
          customer_data_[1] = 3;
          break;
    case 0:
      request = pho_robot_loader::REQUEST::CALIBRATION_ADD_POINT;
      break;
    case 1:
      request = pho_robot_loader::REQUEST::CALIBRATION_SET_TO_SCANNER;
       break;
    case 2:
        sleep(5);
      request = pho_robot_loader::REQUEST::INITIALIZE;

      break;
    default:
  //sleep(5);
     // if (request_counter % 2 == 1)
      //  request = pho_robot_loader::REQUEST::SCAN;
     // else
        request = pho_robot_loader::REQUEST::TRAJECTORY;
      break;

  }

  request_counter++;

  // Parse start and end poses in initialization request
  if (request == pho_robot_loader::REQUEST::INITIALIZE) {
    poses_.request.startPose.position.clear();
    poses_.request.endPose.position.clear();


    poses_.request.startPose.position.resize(6);
    poses_.request.endPose.position.resize(6);

    poses_.request.startPose.position[0] = -0.805;
    poses_.request.startPose.position[1] = -1.794;
    poses_.request.startPose.position[2] = -1.36;
    poses_.request.startPose.position[3] = -1.611;
    poses_.request.startPose.position[4] = 1.57;
    poses_.request.startPose.position[5] = -0.679;

    poses_.request.endPose.position[0] = -0.805;
    poses_.request.endPose.position[1] = -1.794;
    poses_.request.endPose.position[2] = -1.36;
    poses_.request.endPose.position[3] = -1.611;
    poses_.request.endPose.position[4] = 1.57;
    poses_.request.endPose.position[5] = -0.679;


    request = pho_robot_loader::REQUEST::TRAJECTORY;
    return pho_robot_loader::REQUEST::INITIALIZE;
  }

  //sleep(4);
  return request;
}

int VIRTUAL_ROBOT::sendMsgHeader(int message_type, int num_of_operations)
{

  return 1;
}

int VIRTUAL_ROBOT::sendTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory, int traj_number, bool is_fine)
{

 // ROS_ERROR("traj number %d TRAJECTORY SIZE %d",traj_number, trajectory.size());
  if (traj_number == 2 || traj_number == 3){
      create_log(trajectory);
  }
  return 1;
}

int VIRTUAL_ROBOT::sendData(int type, int number, std::vector<int> data)
{

  return 1;
}

std::vector <uint8_t> VIRTUAL_ROBOT::createMsgHeader(int operation_type, int operation_number, int size)
{
  std::vector <uint8_t> message_to_robot_header(MESSAGE_SUB_HEADER_SIZE,0);

  message_to_robot_header[0] = operation_type;
  message_to_robot_header[1] = 0;
  message_to_robot_header[2] = 0;
  message_to_robot_header[3] = 0;
  message_to_robot_header[4] = operation_number;
  message_to_robot_header[5] = 0;
  message_to_robot_header[6] = 0;
  message_to_robot_header[7] = 0;
  message_to_robot_header[8] = size;
  message_to_robot_header[9] = 0;
  message_to_robot_header[10] = 0;
  message_to_robot_header[11] = 0;

  return message_to_robot_header;
}

//---------------------------------------------------------------------------------------
//--------------------------------STATE LISTENER IMPLEMENTATION--------------------------
//---------------------------------------------------------------------------------------

void VIRTUAL_ROBOT::stateListenerConnect(std::string robot_ip, int port)
{

  robot_state_.header.frame_id = "base_link";
  robot_state_.name.resize(NUM_OF_JOINTS);
  robot_state_.position.resize(NUM_OF_JOINTS);
  robot_state_.name[0] = "joint_1";
  robot_state_.name[1] = "joint_2";
  robot_state_.name[2] = "joint_3";
  robot_state_.name[3] = "joint_4";
  robot_state_.name[4] = "joint_5";
  robot_state_.name[5] = "joint_6";

}

void VIRTUAL_ROBOT::stateListenerPublish()
{
  uint8_t message_from_robot[STATE_LISTENER_BUFFER_SIZE];
 // std::vector<float> tool_pose_array;

  // Clear robot state positions
  robot_state_.position.clear();
 // tool_pose_array.clear();

  robot_state_.position.resize(6);

  robot_state_.position[0] = 0;
  robot_state_.position[1] = 0;
  robot_state_.position[2] = 0;
  robot_state_.position[3] = 0;
  robot_state_.position[4] = 1.57;
  robot_state_.position[5] = 0;


  // Publish Joint States and Tool Pose on ROS topics
  joint_state_pub_.publish(robot_state_);
    sleep(1);
  //tool_pose_pub_.publish(tool_state_);
}


    void VIRTUAL_ROBOT::create_log(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory) {

        static int iterator = 0;
        std::string file_name = "/home/controller/catkin_ws/src/virtual_robot/photoneo_virtual_robot_module/log/trajectory_" + std::to_string(iterator++) + ".txt";
        std::ofstream outfile (file_name);

        for (auto point : trajectory) {
            outfile << point.positions[0] << ", ";
            outfile << point.positions[1] << ", ";
            outfile << point.positions[2] << ", ";
            outfile << point.positions[3] << ", ";
            outfile << point.positions[4] << ", ";
            outfile << point.positions[5];
            outfile << ":" << std::endl;
        }

        outfile.close();
    }
} // namespace

PLUGINLIB_EXPORT_CLASS(pho::VIRTUAL_ROBOT, pho_robot_loader::RobotBase)

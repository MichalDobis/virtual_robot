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
#include "../../../../devel/include/photoneo_msgs/initialize_pose.h"

namespace pho {

    VIRTUAL_ROBOT::VIRTUAL_ROBOT() : pho_robot_loader::RobotBase(),
                                     outfile_diferencial(
                                             "/home/controller/catkin_ws/src/virtual_robot/photoneo_virtual_robot_module/diferencial_log/trajectory.txt") {


        if (mkdir("/home/controller/catkin_ws/src/virtual_robot/photoneo_virtual_robot_module/log", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1 && errno != EEXIST)
        {
            ROS_WARN("Error creating log directory %s", strerror(errno));

        } else ROS_DEBUG("Directory created");


        if (mkdir("/home/controller/catkin_ws/src/virtual_robot/photoneo_virtual_robot_module/diferencial_log", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == -1 && errno != EEXIST)
        {
            ROS_WARN("Error creating log directory %s", strerror(errno));

        } else ROS_DEBUG("Directory created");


        number_of_fails = 0;
        attempt = 0;

        ros::NodeHandle nh;
        nh.param<double>("/virtual_robot/cycle_time", cycle_time_, 30);
        nh.param<double>("/virtual_robot/moving_robot_time", moving_robot_time_, 2);

    }

    VIRTUAL_ROBOT::~VIRTUAL_ROBOT() {

    }

    bool VIRTUAL_ROBOT::initialize(std::string robot_ip, int port) {
        initialized_moveit = false;
        return true;
    }

    bool VIRTUAL_ROBOT::connectOrAccept() {

        return true;
    }

    int VIRTUAL_ROBOT::receiveRequestFromRobot() {

        int request = getRequest();

        // Parse start and end poses in initialization request
        if (request == pho_robot_loader::REQUEST::INITIALIZE) {
            poses_.request.startPose.position.clear();
            poses_.request.endPose.position.clear();

            poses_.request.startPose.position.resize(6);
            poses_.request.endPose.position.resize(6);

            ros::NodeHandle nh;

            poses_.request.startPose.position[0] = 0.0;
            poses_.request.startPose.position[1] = 0.0;
            poses_.request.startPose.position[2] = 0.0;
            poses_.request.startPose.position[3] = 0.0;
            poses_.request.startPose.position[4] = 1.57;
            poses_.request.startPose.position[5] = 0;

            poses_.request.endPose.position[0] = 0;
            poses_.request.endPose.position[1] = 0;
            poses_.request.endPose.position[2] = 0;
            poses_.request.endPose.position[3] = 0;
            poses_.request.endPose.position[4] = 1.57;
            poses_.request.endPose.position[5] = 0;


            nh.getParam("/virtual_robot/start_pose", poses_.request.startPose.position);
            nh.getParam("/virtual_robot/end_pose", poses_.request.endPose.position);

            /*poses_.request.startPose.position[0] = -0.805;
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
            poses_.request.endPose.position[5] = -0.679;*/

            initialized_moveit = true;
        }

        return request;
    }

    int VIRTUAL_ROBOT::sendMsgHeader(int message_type, int num_of_operations) {

        return 1;
    }

    int VIRTUAL_ROBOT::sendTrajectory(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory, int traj_number,
                                      bool is_fine) {

        if (traj_number == 2 || traj_number == 3) {
            create_log(trajectory);
        }

        return 1;
    }

    int VIRTUAL_ROBOT::sendData(int type, int number, std::vector<int> data) {


        if (type == pho_robot_loader::OPERATION::ERROR && data[0] == pho_robot_loader::ERROR::SERVICE_ERROR) {
            initialized_moveit = false;
            ROS_INFO("Attempt %d number of fails %d", attempt, number_of_fails++);
            sleep(1);
        }
        return 1;
    }

    std::vector<uint8_t> VIRTUAL_ROBOT::createMsgHeader(int operation_type, int operation_number, int size) {
        std::vector<uint8_t> message_to_robot_header(MESSAGE_SUB_HEADER_SIZE, 0);

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

    void VIRTUAL_ROBOT::stateListenerConnect(std::string robot_ip, int port) {

        robot_state_.header.frame_id = "base_link";
        robot_state_.name.resize(NUM_OF_JOINTS);
        robot_state_.position.resize(NUM_OF_JOINTS);
        robot_state_.name[0] = "joint_1";
        robot_state_.name[1] = "joint_2";
        robot_state_.name[2] = "joint_3";
        robot_state_.name[3] = "joint_4";
        robot_state_.name[4] = "joint_5";
        robot_state_.name[5] = "joint_6";

        poses_.request.startPose.position.resize(6);
        poses_.request.endPose.position.resize(6);

        poses_.request.startPose.position[0] = -0.805;
        poses_.request.startPose.position[1] = -1.794;
        poses_.request.startPose.position[2] = -1.36;
        poses_.request.startPose.position[3] = -1.611;
        poses_.request.startPose.position[4] = 1.57;
        poses_.request.startPose.position[5] = -0.679;

    }

    void VIRTUAL_ROBOT::stateListenerPublish() {
        uint8_t message_from_robot[STATE_LISTENER_BUFFER_SIZE];
        // std::vector<float> tool_pose_array;

        // Clear robot state positions
        robot_state_.position.clear();
        // tool_pose_array.clear();

        robot_state_.position = poses_.request.startPose.position;


        // Publish Joint States and Tool Pose on ROS topics
        joint_state_pub_.publish(robot_state_);
        sleep(1);
        //tool_pose_pub_.publish(tool_state_);
    }


    void VIRTUAL_ROBOT::create_log(std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory) {

        bool badTrajectory = false;
        static int bad_trajectory_counter = 0;

        // Check trajectory
        double limit = 0.5;
        for (int pointIdx = 1; pointIdx < trajectory.size(); pointIdx++) {

            for (int j = 0; j < 6; j++) {
                if (fabs(trajectory[pointIdx].positions[j] -
                                 trajectory[pointIdx - 1].positions[j]) > limit){
                    badTrajectory = true;
                    //bad_trajectory_counter++;
                }
            }
        }

        // Log only bad trajectory
        if (badTrajectory) {

            std::ofstream outfile("/home/controller/catkin_ws/src/virtual_robot/photoneo_virtual_robot_module/log/trajectory_" + std::to_string(bad_trajectory_counter++) + ".txt");
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

        // Log all trajectory
            for (int i = 1; i < trajectory.size(); i++) {
                outfile_diferencial << trajectory[i].positions[0] - trajectory[i - 1].positions[0] << ", ";
                outfile_diferencial << trajectory[i].positions[1] - trajectory[i - 1].positions[1] << ", ";
                outfile_diferencial << trajectory[i].positions[2] - trajectory[i - 1].positions[2] << ", ";
                outfile_diferencial << trajectory[i].positions[3] - trajectory[i - 1].positions[3] << ", ";
                outfile_diferencial << trajectory[i].positions[4] - trajectory[i - 1].positions[4] << ", ";
                outfile_diferencial << trajectory[i].positions[5] - trajectory[i - 1].positions[5] << ", ";
                outfile_diferencial << std::endl;
            }

        ROS_INFO("Attempt %d number of fails %d", attempt++, number_of_fails);
        if (attempt > 20000) {
            outfile_diferencial.close();
            ros::shutdown();
        }

    }

    uint8_t VIRTUAL_ROBOT::getRequest() {

        uint8_t req;
        static int cycle_iterator = 0;

        if (!initialized_moveit) {
            req = pho_robot_loader::REQUEST::INITIALIZE;
            cycle_iterator = 0;

        } else {
            if (cycle_iterator % 2 == 0){

                usleep((moving_robot_time_)*1000000);
                req = pho_robot_loader::REQUEST::SCAN;

            } else {

                usleep((cycle_time_ - moving_robot_time_)*1000000);
                req = pho_robot_loader::REQUEST::TRAJECTORY;

            }

            cycle_iterator++;
        }
        return req;
    }
}

PLUGINLIB_EXPORT_CLASS(pho::VIRTUAL_ROBOT, pho_robot_loader::RobotBase)

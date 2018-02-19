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

#include <ros/ros.h>
//#include <tinyxml.h>
//#include <termios.h>
//#include <boost/thread/thread.hpp>
//#include <sensor_msgs/JointState.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>
//#include <signal.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem/fstream.hpp>
//#include <iostream>     // std::cout
//#include <fstream>      // std::ifstream



class TrajectoryStatistics{
public:

    TrajectoryStatistics() {

    }

    void load_file(std::string file){

        std::ifstream myfile(file,  std::ifstream::in);
        std::string line;
        data.clear();

        if (myfile.is_open())
        {
            std::vector<double> joints_old;

            bool isFirst = true;
            bool addPoint = true;

            while ( std::getline (myfile,line) )
            {
                std::vector<std::string> strs;
                strs.clear();
                boost::split(strs, line, boost::is_any_of(","));

                std::vector<double> joints;
                joints.resize(6);
                if (strs.size() >= 6) {
                    for (int i = 0; i < 6; i++) {
                        joints[i] = std::stof(strs[i]);
                    }
                    data.push_back(joints);
                }
             }
            myfile.close();

        } else{
            ROS_ERROR("file sa neotvoril");
        }

        data.erase(std::unique(data.begin(), data.end()), data.end());
        ROS_INFO("Loaded %d waypoints", data.size());

    }

    void computeSummary(){

        outliers.resize(data.size(), 0);

        for (int i = 0; i < 6; i++){
            computeSummary(i);
        }

        int idx = 0;
        int bad_trajectories_count = 0;

        for (auto count_of_outliers : outliers){
            if (count_of_outliers != 0) {
                ROS_INFO("On waypoint %d is %d outliers", idx, count_of_outliers);
                bad_trajectories_count++;
            }
            idx++;
        }
        ROS_WARN("Bad trajectories: %d", bad_trajectories_count);
    }

    void computeSummary(int joint_idx){

        double number_of_results = data.size();
        std::vector<double> joints;
        joints.clear();

        //compute average
        double sum_joint_diff = 0;
        for (auto joints_diff : data) {
            joints.push_back(joints_diff[joint_idx]);
            sum_joint_diff += joints_diff[joint_idx];
        }
        double average = sum_joint_diff / number_of_results;

        //Sort and get median
        std::sort(joints.begin(), joints.end());
        double median = joints[number_of_results/2];

        double sum_square_diff = 0;
        for (auto joints_diff : data) {
            sum_square_diff += pow(joints_diff[joint_idx] - median, 2);
        }
        double variance = sum_square_diff / (number_of_results - 1);
        double sigma = sqrt(variance);

        const double gain = 10;
        int outliers_count = 0;
        int idx = 0;

        for (auto joints_diff : data) {
            if (fabs(joints_diff[joint_idx] - median) > gain * sigma) {
                outliers_count++;
                outliers[idx]++;
           //     ROS_INFO("On waypoint %d founded diff %f, outlier number %d",idx, fabs(joints_diff[joint_idx] - median), outliers[idx]);
            }
            idx++;
        }

        ROS_WARN("For joint_%d computed sigma is %f and %d outliers", joint_idx + 1, sigma, outliers_count);
    }

private:
    std::vector<std::vector<double>> data;
    std::vector<int> outliers;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_statistics");
    ros::NodeHandle nh("~");

    TrajectoryStatistics test;
    test.load_file("/home/controller/catkin_ws/src/virtual_robot/photoneo_virtual_robot_module/diferencial_log/trajectory_test_lin_3.txt");
    test.computeSummary();

    return 0;
}

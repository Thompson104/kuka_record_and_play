/*
 * Software License Agreement (BSD License)
 *
 *   Copyright (c) 2016, Manuel Bonilla (josemanuelbonilla@gmail.com)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <ros/console.h>
#include <ros/common.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

double gripper;

std::ifstream myfile;

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "record");

    std::string file_name("record.txt");

    if (argc > 1)
    {
        file_name = std::string(argv[1]);
    }

    myfile.open (file_name);
    gripper  = 0.0;

    //Declare the node handle
    ros::NodeHandle node;
    // joints.resize(7);
    ros::Rate rate(100);
    ros::Publisher p_robot = node.advertise<trajectory_msgs::JointTrajectory>("/right_arm/joint_trajectory_controller/command", 1);
    ros::Publisher p_gripper = node.advertise<trajectory_msgs::JointTrajectory>("/gripper/joint_trajectory_controller/command", 1);

    std::string line;
    int a = 1;
    double gripper;
    // ros::spinOnce();

    while (ros::ok() && (a == 1))
    {

        std::vector<double> v(7);
        int i = 0;
        if ( getline (myfile, line, '\n'))
        {
            std::istringstream iss_line(line);
            for (std::string value; getline(iss_line, value, ' ' ); )
            {

                if (i > 6)
                {
                    gripper = std::stod(value);
                    break;
                }
                else
                {
                    v[i] = std::stod(value);
                }
                i++;
            }
        }
        else
        {
            a = 2;
            continue;
        }

        for (int j = 0; j < 7; ++j)
        {
            std::cout << v[j] << " " ;
        }
        std::cout << gripper << std::endl ;

        trajectory_msgs::JointTrajectory robot_msg, gripper_msg;
        gripper_msg.joint_names.push_back(std::string("right_hand_synergy_joint"));

        trajectory_msgs::JointTrajectoryPoint point_n;
        double gripper_max(0.5);
        if (gripper > gripper_max)
            point_n.positions.push_back(gripper_max
                                       );
        else
            point_n.positions.push_back(gripper);
        point_n.time_from_start = ros::Duration(1.0);
        gripper_msg.points.push_back(point_n);

        robot_msg.joint_names.push_back("right_arm_a1_joint");
        robot_msg.joint_names.push_back("right_arm_a2_joint");
        robot_msg.joint_names.push_back("right_arm_a3_joint");
        robot_msg.joint_names.push_back("right_arm_a4_joint");
        robot_msg.joint_names.push_back("right_arm_a5_joint");
        robot_msg.joint_names.push_back("right_arm_a6_joint");
        robot_msg.joint_names.push_back("right_arm_e1_joint");

        trajectory_msgs::JointTrajectoryPoint point_nr;
        for (int i = 0; i < 7; ++i)
        {
            point_nr.positions.push_back(v[i]);
        }
        point_nr.time_from_start = ros::Duration(1.0);
        robot_msg.points.push_back(point_nr);
        std::cout << "Publishng";
        p_gripper.publish(gripper_msg);
        p_robot.publish(robot_msg);
        ros::spinOnce();

        rate.sleep();
    }

    myfile.close();
    return 0;
}

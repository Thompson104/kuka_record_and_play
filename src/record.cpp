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
#include <ros/console.h>
#include <ros/common.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <iostream>
#include <fstream>

double gripper;

std::ofstream myfile;

void get_js(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i=0; i<msg->position.size();++i)
    {
        myfile << msg->position[i] << " ";
    }
    myfile <<  gripper;
    myfile << std::endl;

}

void get_js_gripper(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    gripper = msg->points[0].positions[0];
}

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
    ros::Rate rate(1000);
    ros::Subscriber js = node.subscribe("/right_arm/joint_states", 1, get_js);
    ros::Subscriber js_gripper = node.subscribe("/gripper/joint_trajectory_controller/command", 1, get_js_gripper);

    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }

    myfile.close();
    return 0;
}

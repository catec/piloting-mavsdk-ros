/*!
 *      @file  main.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/4/2021
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2021, FADA-CATEC
 */

#include <mavsdk_ros/mavsdk_ros_node.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mavsdk_ros");
    if (!ros::master::check()) {
        ROS_ERROR("There is not ros master");
        return -1;
    }
    ros::start();

    mavsdk_ros::MavsdkRosNode node;
    if (!node.init())
        return 0;

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
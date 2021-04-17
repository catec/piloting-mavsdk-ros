/*!
 *      @file  parameters.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/4/2021
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2021, FADA-CATEC
 */

#include <ros/ros.h>

#include <mavsdk_ros/parameters.h>

namespace mavsdk_ros {
Parameters::Parameters()
{
    if (!ros::param::get("~local_system_id", local_system_id)) {
        exitWithParameterError("local_system_id");
    }

    if (!ros::param::get("~local_component_id", local_component_id)) {
        exitWithParameterError("local_component_id");
    }

    if (!ros::param::get("~sensor_components_ids", sensor_components_ids)) {
        exitWithParameterError("sensor_components_ids");
    }

    if (!ros::param::get("~local_ip", local_ip)) {
        exitWithParameterError("local_ip");
    }

    if (!ros::param::get("~target_ip", target_ip)) {
        exitWithParameterError("target_ip");
    }

    if (!ros::param::get("~local_port", local_port)) {
        exitWithParameterError("local_port");
    }

    if (!ros::param::get("~target_port", target_port)) {
        exitWithParameterError("target_port");
    }

    if (!ros::param::get("~target_system_id", target_system_id)) {
        exitWithParameterError("target_system_id");
    }
}

void Parameters::exitWithParameterError(const char* parameterStr)
{
    ROS_ERROR("[%s] `%s` parameter not set!", ros::this_node::getName().data(), parameterStr);
    exit(EXIT_FAILURE);
}
} // namespace mavsdk_ros
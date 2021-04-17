/*!
 *      @file  mavsdk_ros_node.h
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/4/2021
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2021, FADA-CATEC
 */

#pragma once

#include <ros/ros.h>

#include <mavsdk_ros/mavsdk_include.h>

namespace mavsdk_ros {
class MavsdkRosNode {
public:
    MavsdkRosNode();
    ~MavsdkRosNode();

    bool init();

private:
    ros::NodeHandle _nh;

    std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
    std::shared_ptr<mavsdk::TelemetryRoboticVehicle> _telemetry;
    std::shared_ptr<mavsdk::InspectionRoboticVehicle> _inspection;
    std::shared_ptr<mavsdk::AlarmRoboticVehicle> _alarm;
    std::shared_ptr<mavsdk::ChecklistRoboticVehicle> _checklist;
    std::shared_ptr<mavsdk::CommandRoboticVehicle> _command;
    std::shared_ptr<mavsdk::HLActionRoboticVehicle> _hl_action;
};
} // namespace mavsdk_ros
/*!
 *      @file  mavsdk_ros_node.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/4/2021
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2021, FADA-CATEC
 */

#include <mavsdk_ros/parameters.h>

#include <mavsdk_ros/mavsdk_ros_node.h>

namespace mavsdk_ros {
MavsdkRosNode::MavsdkRosNode() : _nh("~") {}
MavsdkRosNode::~MavsdkRosNode() {}

bool MavsdkRosNode::init()
{
    Parameters params;

    _mavsdk = std::make_shared<mavsdk::Mavsdk>();
    mavsdk::Mavsdk::Configuration config(
        params.local_system_id,
        params.local_component_id,
        false,
        mavsdk::Mavsdk::Configuration::UsageType::RoboticVehicle);
    _mavsdk->set_configuration(config);

    const auto connection_result =
        _mavsdk->setup_udp_connection(params.local_ip, params.local_port, params.target_ip, params.target_port);

    if (connection_result != mavsdk::ConnectionResult::Success) {
        ROS_ERROR("MAVSDK connection error. Terminating...");
        return false;
    }

    ros::Time begin = ros::Time::now();
    std::shared_ptr<mavsdk::System> target_system;
    while (!(target_system = _mavsdk->system(params.target_system_id))) {
        ROS_INFO_THROTTLE(1, "No target system (%d) alive", params.target_system_id);

        const double elapsed_time = (ros::Time::now() - begin).toSec();
        if (elapsed_time >= 30.0) {
            ROS_ERROR("No target system (%d) found in 30 seconds. Terminating...");
            return false;
        }

        ros::Duration(0.25).sleep();
    }

    initAlarm(target_system);
    initCommand(target_system);
    initChecklist(target_system);
    initHLAction(target_system);

    _telemetry  = std::make_shared<mavsdk::TelemetryRoboticVehicle>(target_system);
    _inspection = std::make_shared<mavsdk::InspectionRoboticVehicle>(target_system);

    return true;
}

void MavsdkRosNode::initAlarm(std::shared_ptr<mavsdk::System>& target_system)
{
    _alarm = std::make_shared<mavsdk::AlarmRoboticVehicle>(target_system);

    _alarm_status_sub = _nh.subscribe<mavsdk_ros::AlarmStatus>("alarm_status", 10, &MavsdkRosNode::alarmStatusCb, this);

    _set_upload_alarm_srv = _nh.advertiseService("set_upload_alarm", &MavsdkRosNode::setUploadAlarmCb, this);
    _upload_alarm_srv     = _nh.advertiseService("upload_alarm", &MavsdkRosNode::uploadAlarmCb, this);
}

void MavsdkRosNode::initCommand(std::shared_ptr<mavsdk::System>& target_system)
{
    _command = std::make_shared<mavsdk::CommandRoboticVehicle>(target_system);

    _commands_ack_sub = _nh.subscribe<mavsdk_ros::CommandAck>("commands_ack", 10, &MavsdkRosNode::commandsAckCb, this);

    _received_commands_pub = _nh.advertise<mavsdk_ros::CommandLong>("commands", 10);
    _command->subscribe_command([&](mavsdk::CommandBase::CommandLong cmd) {
        mavsdk_ros::CommandLong command_msg;
        command_msg.command      = cmd.command;
        command_msg.confirmation = cmd.confirmation;
        command_msg.param1       = cmd.params.param1;
        command_msg.param2       = cmd.params.param2;
        command_msg.param3       = cmd.params.param3;
        command_msg.param4       = cmd.params.param4;
        command_msg.param5       = cmd.params.param5;
        command_msg.param6       = cmd.params.param6;
        command_msg.param7       = cmd.params.param7;

        _received_commands_pub.publish(command_msg);
    });
}

void MavsdkRosNode::initChecklist(std::shared_ptr<mavsdk::System>& target_system)
{
    _checklist = std::make_shared<mavsdk::ChecklistRoboticVehicle>(target_system);

    _set_upload_checklist_srv =
        _nh.advertiseService("set_upload_checklist", &MavsdkRosNode::setUploadChecklistCb, this);
    _upload_checklist_srv = _nh.advertiseService("upload_checklist", &MavsdkRosNode::uploadChecklistCb, this);
}

void MavsdkRosNode::initHLAction(std::shared_ptr<mavsdk::System>& target_system)
{
    _hl_action = std::make_shared<mavsdk::HLActionRoboticVehicle>(target_system);

    _set_upload_hl_action_srv = _nh.advertiseService("set_upload_hl_action", &MavsdkRosNode::setUploadHLActionCb, this);
    _upload_hl_action_srv     = _nh.advertiseService("upload_hl_action", &MavsdkRosNode::uploadHLActionCb, this);
}

} // namespace mavsdk_ros
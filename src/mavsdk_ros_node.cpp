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

#include <std_msgs/UInt16.h>

#include <mavsdk_ros/mavsdk_ros_node.h>

namespace mavsdk_ros {
MavsdkRosNode::MavsdkRosNode() : _nh("~") {}
MavsdkRosNode::~MavsdkRosNode() {}

bool MavsdkRosNode::init()
{
    _mavsdk = std::make_shared<mavsdk::Mavsdk>();
    mavsdk::Mavsdk::Configuration config(
        _params.local_system_id,
        _params.local_component_id,
        false,
        mavsdk::Mavsdk::Configuration::UsageType::RoboticVehicle);
    _mavsdk->set_configuration(config);

    const auto connection_result =
        _mavsdk->setup_udp_connection(_params.local_ip, _params.local_port, _params.target_ip, _params.target_port);

    if (connection_result != mavsdk::ConnectionResult::Success) {
        ROS_ERROR("MAVSDK connection error. Terminating...");
        return false;
    }

    // Sleep to let simulation time work
    ros::Duration(1.0).sleep();

    ros::Time begin = ros::Time::now();
    std::shared_ptr<mavsdk::System> target_system;
    while (!(target_system = _mavsdk->system(_params.target_system_id))) {
        ROS_WARN_THROTTLE(1, "No target system (%d) alive", _params.target_system_id);

        double elapsed_time = (ros::Time::now() - begin).toSec();
        if (elapsed_time >= 30.0) {
            ROS_ERROR("No target system (%d) found in %.2f seconds. Terminating...", _params.target_system_id, elapsed_time);
            return false;
        }

        ros::Duration(0.25).sleep();
    }

    initAlarm(target_system);
    initCommand(target_system);
    initChecklist(target_system);
    initHLAction(target_system);
    initInspection(target_system);
    initTelemetry(target_system);

    return true;
}

void MavsdkRosNode::initAlarm(std::shared_ptr<mavsdk::System>& target_system)
{
    _alarm = std::make_shared<mavsdk::AlarmRoboticVehicle>(target_system);

    _alarm_status_sub = _nh.subscribe<mavsdk_ros::AlarmStatus>("alarm_status", 10, &MavsdkRosNode::alarmStatusCb, this);

    mavsdk::AlarmBase::AlarmList alarm_list_empty;
    _alarm->upload_alarm_async(
        [&](mavsdk::AlarmBase::Result result, mavsdk::AlarmBase::Ack ack) {
            ROS_INFO_STREAM("Alarm upload callback. Result [" << result << "] Ack [" << ack << "]");
        },
        alarm_list_empty);

    _set_upload_alarm_srv = _nh.advertiseService("set_upload_alarm", &MavsdkRosNode::setUploadAlarmCb, this);
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
    
    mavsdk::ChecklistBase::Checklist checklist_list_empty;
    _checklist->upload_checklist_async(
        [&](mavsdk::ChecklistBase::Result result, mavsdk::ChecklistBase::Ack ack) {
            ROS_INFO_STREAM("Checklist upload callback. Result [" << result << "] Ack [" << ack << "]");
        },
        checklist_list_empty);

    _set_upload_checklist_srv =
        _nh.advertiseService("set_upload_checklist", &MavsdkRosNode::setUploadChecklistCb, this);
}

void MavsdkRosNode::initHLAction(std::shared_ptr<mavsdk::System>& target_system)
{
    _hl_action = std::make_shared<mavsdk::HLActionRoboticVehicle>(target_system);

    mavsdk::HLActionBase::HLActionList hl_action_list_empty;
    _hl_action->upload_hl_action_async(
        [&](mavsdk::HLActionBase::Result result, mavsdk::HLActionBase::Ack ack) {
            ROS_INFO_STREAM("HLAction upload callback. Result [" << result << "] Ack [" << ack << "]");
        },
        hl_action_list_empty);

    _set_upload_hl_action_srv = _nh.advertiseService("set_upload_hl_action", &MavsdkRosNode::setUploadHLActionCb, this);
}

void MavsdkRosNode::initInspection(std::shared_ptr<mavsdk::System>& target_system)
{
    _inspection = std::make_shared<mavsdk::InspectionRoboticVehicle>(target_system);

    _received_inspection_set_current_pub = _nh.advertise<std_msgs::UInt16>("inspection_set_current", 10);
    _downloaded_inspection_wp_list_pub = _nh.advertise<mavsdk_ros::WaypointList>("inspection_wp_list", 10, true);

    _inspection->subscribe_inspection_set_current([&](uint16_t seq) {
        std_msgs::UInt16 set_current_msg;
        set_current_msg.data = seq;

        _received_inspection_set_current_pub.publish(set_current_msg);
    });

    mavsdk::InspectionBase::WaypointList waypoint_list_emtpy;
    _inspection->upload_inspection_async(
        [&](mavsdk::InspectionBase::Result result, mavsdk::InspectionBase::Ack ack) {
            ROS_INFO_STREAM("Inspection upload callback. Result [" << result << "] Ack [" << ack << "]");
        },
        waypoint_list_emtpy);

    _inspection->download_inspection_async(
        [&](mavsdk::InspectionBase::Result result, mavsdk::InspectionBase::WaypointList waypoint_list) {
            ROS_INFO_STREAM(
                "Inspection download callback. Result [" << result << "] Waypoint list size ["
                                                         << waypoint_list.items.size() << "]");

            if (result == mavsdk::InspectionBase::Result::Success) {
                mavsdk_ros::WaypointList waypoint_list_msg;
                for (auto waypoint_item_base : waypoint_list.items) {
                    mavsdk_ros::WaypointItem waypoint_item;
                    waypoint_item.task_id      = waypoint_item_base.task_id;
                    waypoint_item.command      = waypoint_item_base.command;
                    waypoint_item.autocontinue = waypoint_item_base.autocontinue;
                    waypoint_item.param1       = waypoint_item_base.param1;
                    waypoint_item.param2       = waypoint_item_base.param2;
                    waypoint_item.param3       = waypoint_item_base.param3;
                    waypoint_item.param4       = waypoint_item_base.param4;
                    waypoint_item.x            = waypoint_item_base.x;
                    waypoint_item.y            = waypoint_item_base.y;
                    waypoint_item.z            = waypoint_item_base.z;
                    waypoint_list_msg.items.push_back(waypoint_item);
                }

                _downloaded_inspection_wp_list_pub.publish(waypoint_list_msg);
            }
        });

    // clang-format off
    _set_upload_waypoint_list_srv =
        _nh.advertiseService("set_upload_waypoint_list", &MavsdkRosNode::setUploadWaypointListCb, this);
    _update_current_waypoint_item_srv =
        _nh.advertiseService("update_current_waypoint_item", &MavsdkRosNode::updateCurrentWaypointItemCb, this);
    _update_reached_waypoint_item_srv =
        _nh.advertiseService("update_reached_waypoint_item", &MavsdkRosNode::updateReachedWaypointItemCb, this);
    // clang-format on
}

void MavsdkRosNode::initTelemetry(std::shared_ptr<mavsdk::System>& target_system)
{
    _telemetry = std::make_shared<mavsdk::TelemetryRoboticVehicle>(target_system);

    _text_status_sub = _nh.subscribe<mavsdk_ros::TextStatus>("text_status", 10, &MavsdkRosNode::textStatusCb, this);

    ros::NodeHandle n;

    _pose_sub.subscribe(n, "mavros/local_position/pose", 1);
    _vel_sub.subscribe(n, "mavros/local_position/velocity_local", 1);
    _sync.reset(new Sync(MySyncPolicy(10), _pose_sub, _vel_sub));
    _sync->registerCallback(boost::bind(&MavsdkRosNode::telemetryCb, this, _1, _2));
}

} // namespace mavsdk_ros

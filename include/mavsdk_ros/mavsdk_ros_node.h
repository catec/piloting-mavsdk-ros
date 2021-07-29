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
#include <mavsdk_ros/parameters.h>

// msgs
#include <mavsdk_ros/AlarmStatus.h>
#include <mavsdk_ros/CommandLong.h>
#include <mavsdk_ros/CommandAck.h>
#include <mavsdk_ros/WaypointList.h>
#include <mavsdk_ros/TextStatus.h>

// srvs
#include <mavsdk_ros/SetUploadAlarm.h>
#include <mavsdk_ros/SetUploadChecklist.h>
#include <mavsdk_ros/SetUploadHLAction.h>
#include <mavsdk_ros/SetUploadWaypointList.h>
#include <mavsdk_ros/UpdateSeqWaypointItem.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace mavsdk_ros {
class MavsdkRosNode {
public:
    MavsdkRosNode();
    ~MavsdkRosNode();

    bool init();

private:
    void initAlarm(std::shared_ptr<mavsdk::System>& target_system);
    void initCommand(std::shared_ptr<mavsdk::System>& target_system);
    void initChecklist(std::shared_ptr<mavsdk::System>& target_system);
    void initHLAction(std::shared_ptr<mavsdk::System>& target_system);
    void initInspection(std::shared_ptr<mavsdk::System>& target_system);
    void initTelemetry(std::shared_ptr<mavsdk::System>& target_system);

    void alarmStatusCb(const mavsdk_ros::AlarmStatus::ConstPtr& msg);
    void commandsAckCb(const mavsdk_ros::CommandAck::ConstPtr& msg);
    void telemetryCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, const geometry_msgs::TwistStamped::ConstPtr& velocity_msg);
    void textStatusCb(const mavsdk_ros::TextStatus::ConstPtr& msg);
    
    // clang-format off
    bool setUploadAlarmCb(mavsdk_ros::SetUploadAlarm::Request& request,
                          mavsdk_ros::SetUploadAlarm::Response& response);
    bool setUploadChecklistCb(mavsdk_ros::SetUploadChecklist::Request& request,
                              mavsdk_ros::SetUploadChecklist::Response& response);
    bool setUploadHLActionCb(mavsdk_ros::SetUploadHLAction::Request& request,
                             mavsdk_ros::SetUploadHLAction::Response& response);
    bool setUploadWaypointListCb(mavsdk_ros::SetUploadWaypointList::Request& request,
                                 mavsdk_ros::SetUploadWaypointList::Response& response);
    bool updateCurrentWaypointItemCb(mavsdk_ros::UpdateSeqWaypointItem::Request& request,
                                     mavsdk_ros::UpdateSeqWaypointItem::Response& response);
    bool updateReachedWaypointItemCb(mavsdk_ros::UpdateSeqWaypointItem::Request& request,
                                     mavsdk_ros::UpdateSeqWaypointItem::Response& response);
    // clang-format on

    ros::NodeHandle _nh;
    Parameters _params;

    std::shared_ptr<mavsdk::Mavsdk> _mavsdk;
    std::shared_ptr<mavsdk::TelemetryRoboticVehicle> _telemetry;
    std::shared_ptr<mavsdk::InspectionRoboticVehicle> _inspection;
    std::shared_ptr<mavsdk::AlarmRoboticVehicle> _alarm;
    std::shared_ptr<mavsdk::ChecklistRoboticVehicle> _checklist;
    std::shared_ptr<mavsdk::CommandRoboticVehicle> _command;
    std::shared_ptr<mavsdk::HLActionRoboticVehicle> _hl_action;

    // ROS Services
    ros::ServiceServer _set_upload_alarm_srv;
    ros::ServiceServer _set_upload_checklist_srv;
    ros::ServiceServer _set_upload_hl_action_srv;
    ros::ServiceServer _set_upload_waypoint_list_srv;
    ros::ServiceServer _update_current_waypoint_item_srv;
    ros::ServiceServer _update_reached_waypoint_item_srv;

    // ROS Publishers
    ros::Publisher _received_commands_pub;
    ros::Publisher _received_inspection_set_current_pub;
    ros::Publisher _downloaded_inspection_wp_list_pub;

    // ROS Subscribers
    ros::Subscriber _alarm_status_sub;
    ros::Subscriber _commands_ack_sub;
    ros::Subscriber _text_status_sub;
    message_filters::Subscriber<geometry_msgs::PoseStamped> _pose_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped> _vel_sub;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> _sync;
};
} // namespace mavsdk_ros

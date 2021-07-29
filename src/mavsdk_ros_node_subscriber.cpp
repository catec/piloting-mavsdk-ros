/*!
 *      @file  mavsdk_ros_node_subscriber.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  17/4/2021
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2021, FADA-CATEC
 */

#include <mavsdk_ros/mavsdk_ros_node.h>

namespace mavsdk_ros {

void MavsdkRosNode::alarmStatusCb(const mavsdk_ros::AlarmStatus::ConstPtr& msg)
{
    mavsdk::AlarmBase::AlarmStatus alarm_status;
    alarm_status.stamp_ms = msg->stamp.toNSec() * 1e-6;
    alarm_status.index    = msg->index;

    if (msg->status == mavsdk_ros::AlarmStatus::OK)
        alarm_status.status = mavsdk::AlarmBase::AlarmStatusType::Ok;
    else if (msg->status == mavsdk_ros::AlarmStatus::WARNING)
        alarm_status.status = mavsdk::AlarmBase::AlarmStatusType::Warning;
    else
        alarm_status.status = mavsdk::AlarmBase::AlarmStatusType::Error;

    alarm_status.errors_count = msg->errors_count;
    alarm_status.warns_count  = msg->warns_count;

    _alarm->send_alarm_status(alarm_status);
}

void MavsdkRosNode::commandsAckCb(const mavsdk_ros::CommandAck::ConstPtr& msg)
{
    mavsdk::CommandBase::CommandAck command_ack;
    command_ack.command       = msg->command;
    command_ack.result        = msg->result;
    command_ack.progress      = msg->progress;
    command_ack.result_param2 = msg->result_param2;

    _command->send_ack(command_ack);
}

void MavsdkRosNode::telemetryCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, const geometry_msgs::TwistStamped::ConstPtr& velocity_msg)
{
    mavsdk::TelemetryBase::PositionVelocityNed vehicle_position;
    vehicle_position.system_id = _params.local_system_id;
    vehicle_position.component_id = _params.local_component_id;
    vehicle_position.stamp_ms = pose_msg->header.stamp.toNSec() * 1e-6;
    vehicle_position.position.north_m = pose_msg->pose.position.y;
    vehicle_position.position.east_m =  pose_msg->pose.position.x;
    vehicle_position.position.down_m = -pose_msg->pose.position.z;
    vehicle_position.velocity.north_m_s = velocity_msg->twist.linear.y;
    vehicle_position.velocity.east_m_s =  velocity_msg->twist.linear.x;
    vehicle_position.velocity.down_m_s = -velocity_msg->twist.linear.z;

    _telemetry->send_local_position_ned(vehicle_position);

    mavsdk::TelemetryBase::Attitude vehicle_attitude;
    vehicle_attitude.system_id = _params.local_system_id;
    vehicle_attitude.component_id = _params.local_component_id;
    vehicle_attitude.stamp_ms = pose_msg->header.stamp.toNSec() * 1e-6;
    vehicle_attitude.quaternion_angle.x =  pose_msg->pose.orientation.y;
    vehicle_attitude.quaternion_angle.y =  pose_msg->pose.orientation.x;
    vehicle_attitude.quaternion_angle.z = -pose_msg->pose.orientation.z;
    vehicle_attitude.quaternion_angle.w =  pose_msg->pose.orientation.w;
    vehicle_attitude.angular_velocity.roll_rad_s =  velocity_msg->twist.angular.y;
    vehicle_attitude.angular_velocity.pitch_rad_s = velocity_msg->twist.angular.x;
    vehicle_attitude.angular_velocity.yaw_rad_s =  -velocity_msg->twist.angular.z;

    _telemetry->send_attitude(vehicle_attitude);
}

void MavsdkRosNode::textStatusCb(const mavsdk_ros::TextStatus::ConstPtr& msg)
{
    mavsdk::TelemetryBase::TextStatus text_status;

    if (msg->type == mavsdk_ros::TextStatus::INFO)
        text_status.type = mavsdk::TelemetryBase::TextStatusType::Info;
    else if (msg->type == mavsdk_ros::TextStatus::WARNING)
        text_status.type = mavsdk::TelemetryBase::TextStatusType::Warning;
    else
        text_status.type = mavsdk::TelemetryBase::TextStatusType::Error;

    text_status.text = msg->text;

    _telemetry->send_text_status(text_status);
}

} // namespace mavsdk_ros
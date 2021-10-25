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

void MavsdkRosNode::telemetryCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    sendTelemetry(msg->header.stamp.toNSec() * 1e-6,
                  msg->pose.pose, msg->twist.twist);
}

void MavsdkRosNode::telemetryCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg,
                                const geometry_msgs::TwistStamped::ConstPtr& twist_msg)
{
    sendTelemetry(pose_msg->header.stamp.toNSec() * 1e-6,
                  pose_msg->pose, twist_msg->twist);
}

void MavsdkRosNode::sendTelemetry(const uint32_t stamp_ms, const geometry_msgs::Pose& pose,
                                  const geometry_msgs::Twist& twist)
{
    mavsdk::TelemetryBase::PositionVelocityNed position;
    position.system_id          =  _params.local_system_id;
    position.component_id       =  _params.local_component_id;
    position.stamp_ms           =  stamp_ms;
    position.position.north_m   =  pose.position.y;
    position.position.east_m    =  pose.position.x;
    position.position.down_m    = -pose.position.z;
    position.velocity.north_m_s =  twist.linear.y;
    position.velocity.east_m_s  =  twist.linear.x;
    position.velocity.down_m_s  = -twist.linear.z;

    _telemetry->send_local_position_ned(position);

    mavsdk::TelemetryBase::Attitude attitude;
    attitude.system_id                    =  _params.local_system_id;
    attitude.component_id                 =  _params.local_component_id;
    attitude.stamp_ms                     =  stamp_ms;
    attitude.quaternion_angle.x           =  pose.orientation.y;
    attitude.quaternion_angle.y           =  pose.orientation.x;
    attitude.quaternion_angle.z           = -pose.orientation.z;
    attitude.quaternion_angle.w           =  pose.orientation.w;
    attitude.angular_velocity.roll_rad_s  =  twist.angular.y;
    attitude.angular_velocity.pitch_rad_s =  twist.angular.x;
    attitude.angular_velocity.yaw_rad_s   = -twist.angular.z;

    _telemetry->send_attitude(attitude);
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

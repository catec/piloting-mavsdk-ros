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

} // namespace mavsdk_ros
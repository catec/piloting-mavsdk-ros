/*!
 *      @file  mavsdk_ros_node_services.cpp
 *    @author  Rafael Caballero González (RCG), rcaballero@catec.aero
 *
 *  @internal
 *    Created  18/4/2021
 *   Compiler  gcc/g++
 *    Company  FADA-CATEC
 *  Copyright (c) 2021, FADA-CATEC
 */

#include <future>

#include <mavsdk_ros/mavsdk_ros_node.h>

namespace mavsdk_ros {

bool MavsdkRosNode::setUploadAlarmCb(
    mavsdk_ros::SetUploadAlarm::Request& request, mavsdk_ros::SetUploadAlarm::Response&)
{
    mavsdk::AlarmBase::AlarmList alarm_list;
    for (auto alarm : request.alarms) {
        mavsdk::AlarmBase::AlarmItem alarm_item;
        alarm_item.index       = alarm.index;
        alarm_item.name        = alarm.name;
        alarm_item.description = alarm.description;
        alarm_list.items.push_back(alarm_item);
    }

    _alarm->set_upload_alarm(alarm_list);

    return true;
}

bool MavsdkRosNode::uploadAlarmCb(
    mavsdk_ros::UploadAlarm::Request& request, mavsdk_ros::UploadAlarm::Response& response)
{
    mavsdk::AlarmBase::AlarmList alarm_list;
    for (auto alarm : request.alarms) {
        mavsdk::AlarmBase::AlarmItem alarm_item;
        alarm_item.index       = alarm.index;
        alarm_item.name        = alarm.name;
        alarm_item.description = alarm.description;
        alarm_list.items.push_back(alarm_item);
    }

    auto prom          = std::make_shared<std::promise<std::pair<mavsdk::AlarmBase::Result, mavsdk::AlarmBase::Ack>>>();
    auto future_result = prom->get_future();

    _alarm->upload_alarm_async(
        [prom](mavsdk::AlarmBase::Result result, mavsdk::AlarmBase::Ack ack) {
            prom->set_value(std::make_pair<>(result, ack));
        },
        alarm_list);

    const auto lambda_result         = future_result.get();
    mavsdk::AlarmBase::Result result = lambda_result.first;
    mavsdk::AlarmBase::Ack ack       = lambda_result.second;

    std::stringstream ss;
    ss << result << " - " << ack;
    response.message = ss.str();

    if (result != mavsdk::AlarmBase::Result::Success)
        response.success = false;

    response.success = true;
    return true;
}

} // namespace mavsdk_ros
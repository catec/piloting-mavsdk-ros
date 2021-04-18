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

bool MavsdkRosNode::setUploadChecklistCb(
    mavsdk_ros::SetUploadChecklist::Request& request, mavsdk_ros::SetUploadChecklist::Response&)
{
    mavsdk::ChecklistBase::Checklist checklist_list;
    for (auto checklist : request.checklist) {
        mavsdk::ChecklistBase::ChecklistItem checklist_item;
        checklist_item.index       = checklist.index;
        checklist_item.name        = checklist.name;
        checklist_item.description = checklist.description;
        checklist_list.items.push_back(checklist_item);
    }

    _checklist->set_upload_checklist(checklist_list);

    return true;
}

bool MavsdkRosNode::uploadChecklistCb(
    mavsdk_ros::UploadChecklist::Request& request, mavsdk_ros::UploadChecklist::Response& response)
{
    mavsdk::ChecklistBase::Checklist checklist_list;
    for (auto checklist : request.checklist) {
        mavsdk::ChecklistBase::ChecklistItem checklist_item;
        checklist_item.index       = checklist.index;
        checklist_item.name        = checklist.name;
        checklist_item.description = checklist.description;
        checklist_list.items.push_back(checklist_item);
    }

    auto prom = std::make_shared<std::promise<std::pair<mavsdk::ChecklistBase::Result, mavsdk::ChecklistBase::Ack>>>();
    auto future_result = prom->get_future();

    _checklist->upload_checklist_async(
        [prom](mavsdk::ChecklistBase::Result result, mavsdk::ChecklistBase::Ack ack) {
            prom->set_value(std::make_pair<>(result, ack));
        },
        checklist_list);

    const auto lambda_result             = future_result.get();
    mavsdk::ChecklistBase::Result result = lambda_result.first;
    mavsdk::ChecklistBase::Ack ack       = lambda_result.second;

    std::stringstream ss;
    ss << result << " - " << ack;
    response.message = ss.str();

    if (result != mavsdk::ChecklistBase::Result::Success)
        response.success = false;

    response.success = true;
    return true;
}

bool MavsdkRosNode::setUploadHLActionCb(
    mavsdk_ros::SetUploadHLAction::Request& request, mavsdk_ros::SetUploadHLAction::Response&)
{
    mavsdk::HLActionBase::HLActionList hl_action_list;
    for (auto hl_action : request.hl_actions) {
        mavsdk::HLActionBase::HLActionItem hl_action_item;
        hl_action_item.index       = hl_action.index;
        hl_action_item.command     = hl_action.command;
        hl_action_item.name        = hl_action.name;
        hl_action_item.description = hl_action.description;
        hl_action_list.items.push_back(hl_action_item);
    }

    _hl_action->set_upload_hl_action(hl_action_list);

    return true;
}

bool MavsdkRosNode::uploadHLActionCb(
    mavsdk_ros::UploadHLAction::Request& request, mavsdk_ros::UploadHLAction::Response& response)
{
    mavsdk::HLActionBase::HLActionList hl_action_list;
    for (auto hl_action : request.hl_actions) {
        mavsdk::HLActionBase::HLActionItem hl_action_item;
        hl_action_item.index       = hl_action.index;
        hl_action_item.command     = hl_action.command;
        hl_action_item.name        = hl_action.name;
        hl_action_item.description = hl_action.description;
        hl_action_list.items.push_back(hl_action_item);
    }

    auto prom = std::make_shared<std::promise<std::pair<mavsdk::HLActionBase::Result, mavsdk::HLActionBase::Ack>>>();
    auto future_result = prom->get_future();

    _hl_action->upload_hl_action_async(
        [prom](mavsdk::HLActionBase::Result result, mavsdk::HLActionBase::Ack ack) {
            prom->set_value(std::make_pair<>(result, ack));
        },
        hl_action_list);

    const auto lambda_result            = future_result.get();
    mavsdk::HLActionBase::Result result = lambda_result.first;
    mavsdk::HLActionBase::Ack ack       = lambda_result.second;

    std::stringstream ss;
    ss << result << " - " << ack;
    response.message = ss.str();

    if (result != mavsdk::HLActionBase::Result::Success)
        response.success = false;

    response.success = true;
    return true;
}

} // namespace mavsdk_ros
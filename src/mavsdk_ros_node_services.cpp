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

    if (result == mavsdk::AlarmBase::Result::Success)
        response.success = true;
    else
        response.success = false;

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

    if (result == mavsdk::ChecklistBase::Result::Success)
        response.success = true;
    else
        response.success = false;

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

    if (result == mavsdk::HLActionBase::Result::Success)
        response.success = true;
    else
        response.success = false;

    return true;
}

bool MavsdkRosNode::setUploadInspectionCb(
    mavsdk_ros::SetUploadInspection::Request& request, mavsdk_ros::SetUploadInspection::Response&)
{
    mavsdk::InspectionBase::InspectionPlan inspection_plan;
    inspection_plan.mission_id = request.mission_id;

    for (auto inspection_item : request.inspection_items) {
        mavsdk::InspectionBase::InspectionItem inspection_item_base;
        inspection_item_base.command      = inspection_item.command;
        inspection_item_base.autocontinue = inspection_item.autocontinue;
        inspection_item_base.param1       = inspection_item.param1;
        inspection_item_base.param2       = inspection_item.param2;
        inspection_item_base.param3       = inspection_item.param3;
        inspection_item_base.param4       = inspection_item.param4;
        inspection_item_base.x            = inspection_item.x;
        inspection_item_base.y            = inspection_item.y;
        inspection_item_base.z            = inspection_item.z;

        inspection_plan.inspection_items.push_back(inspection_item_base);
    }

    _inspection->set_upload_inspection(inspection_plan);

    return true;
}

bool MavsdkRosNode::uploadInspectionCb(
    mavsdk_ros::UploadInspection::Request& request, mavsdk_ros::UploadInspection::Response& response)
{
    mavsdk::InspectionBase::InspectionPlan inspection_plan;
    inspection_plan.mission_id = request.mission_id;

    for (auto inspection_item : request.inspection_items) {
        mavsdk::InspectionBase::InspectionItem inspection_item_base;
        inspection_item_base.command      = inspection_item.command;
        inspection_item_base.autocontinue = inspection_item.autocontinue;
        inspection_item_base.param1       = inspection_item.param1;
        inspection_item_base.param2       = inspection_item.param2;
        inspection_item_base.param3       = inspection_item.param3;
        inspection_item_base.param4       = inspection_item.param4;
        inspection_item_base.x            = inspection_item.x;
        inspection_item_base.y            = inspection_item.y;
        inspection_item_base.z            = inspection_item.z;

        inspection_plan.inspection_items.push_back(inspection_item_base);
    }

    auto prom =
        std::make_shared<std::promise<std::pair<mavsdk::InspectionBase::Result, mavsdk::InspectionBase::Ack>>>();
    auto future_result = prom->get_future();

    _inspection->upload_inspection_async(
        [prom](mavsdk::InspectionBase::Result result, mavsdk::InspectionBase::Ack ack) {
            prom->set_value(std::make_pair<>(result, ack));
        },
        inspection_plan);

    const auto lambda_result              = future_result.get();
    mavsdk::InspectionBase::Result result = lambda_result.first;
    mavsdk::InspectionBase::Ack ack       = lambda_result.second;

    std::stringstream ss;
    ss << result << " - " << ack;
    response.message = ss.str();

    if (result == mavsdk::InspectionBase::Result::Success)
        response.success = true;
    else
        response.success = false;

    return true;
}

bool MavsdkRosNode::downloadInspectionCb(
    mavsdk_ros::DownloadInspection::Request&, mavsdk_ros::DownloadInspection::Response& response)
{
    auto prom = std::make_shared<
        std::promise<std::pair<mavsdk::InspectionBase::Result, mavsdk::InspectionBase::InspectionPlan>>>();
    auto future_result = prom->get_future();

    _inspection->download_inspection_async(
        [prom](mavsdk::InspectionBase::Result result, mavsdk::InspectionBase::InspectionPlan inspection_plan) {
            prom->set_value(std::make_pair<>(result, inspection_plan));
        });

    const auto lambda_result                               = future_result.get();
    mavsdk::InspectionBase::Result result                  = lambda_result.first;
    mavsdk::InspectionBase::InspectionPlan inspection_plan = lambda_result.second;

    std::stringstream ss;
    ss << result;
    response.message = ss.str();

    if (result == mavsdk::InspectionBase::Result::Success) {
        response.success    = true;
        response.mission_id = inspection_plan.mission_id;
        for (auto inspection_item_base : inspection_plan.inspection_items) {
            mavsdk_ros::InspectionItem inspection_item;
            inspection_item.command      = inspection_item_base.command;
            inspection_item.autocontinue = inspection_item_base.autocontinue;
            inspection_item.param1       = inspection_item_base.param1;
            inspection_item.param2       = inspection_item_base.param2;
            inspection_item.param3       = inspection_item_base.param3;
            inspection_item.param4       = inspection_item_base.param4;
            inspection_item.x            = inspection_item_base.x;
            inspection_item.y            = inspection_item_base.y;
            inspection_item.z            = inspection_item_base.z;
            response.inspection_items.push_back(inspection_item);
        }
    } else
        response.success = false;

    return true;
}

} // namespace mavsdk_ros
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

bool MavsdkRosNode::setUploadInspectionCb(
    mavsdk_ros::SetUploadInspection::Request& request, mavsdk_ros::SetUploadInspection::Response&)
{
    mavsdk::InspectionBase::InspectionPlan inspection_plan;
    inspection_plan.mission_id = request.inspection_plan.mission_id;

    for (auto inspection_item : request.inspection_plan.inspection_items) {
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

bool MavsdkRosNode::updateCurrentInspectionItemCb(
    mavsdk_ros::UpdateSeqInspectionItem::Request& request, mavsdk_ros::UpdateSeqInspectionItem::Response&)
{
    uint16_t item_seq = request.item_seq;
    _inspection->update_current_inspection_item(item_seq);

    return true;
}

bool MavsdkRosNode::updateReachedInspectionItemCb(
    mavsdk_ros::UpdateSeqInspectionItem::Request& request, mavsdk_ros::UpdateSeqInspectionItem::Response&)
{
    uint16_t item_seq = request.item_seq;
    _inspection->update_reached_inspection_item(item_seq);

    return true;
}

} // namespace mavsdk_ros
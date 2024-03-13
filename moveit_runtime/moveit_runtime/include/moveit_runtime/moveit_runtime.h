//
// Created by fan on 2021/7/19.
//

#pragma once

#include <string>

namespace moveit_runtime
{
/// move_group
namespace move_group
{
static const std::string NODE_NAME = "move_group";  // name of node
}

/// topic
namespace topic
{
/*
/attached_collision_object
/collision_object
/joint_states
/move_group/display_contacts
/move_group/display_cost_sources
/move_group/display_grasp_markers
/move_group/display_planned_path
/move_group/fake_controller_joint_states
/move_group/monitored_planning_scene
/move_group/motion_plan_request
/planning_scene
/planning_scene_world
/recognized_object_array
/tf
/tf_static
/trajectory_execution_event
*/

static const std::string ATTACHED_COLLISION_OBJECT_TOPIC_NAME = "/attached_collision_object";
static const std::string COLLISION_OBJECT_TOPIC_NAME = "/collision_object";
static const std::string JOINT_STATES_TOPIC_NAME = "/joint_states";
static const std::string INNER_POINT_CLOUD2_TOPIC_NAME = "/inner_point_cloud2"; //内部使用

static const std::string DISPLAY_CONTACTS_TOPIC_NAME = "/move_group/display_contacts";
static const std::string DISPLAY_COST_SOURCES_TOPIC_NAME = "/move_group/display_cost_sources";
static const std::string DISPLAY_GRASP_MARKERS_TOPIC_NAME = "/move_group/display_grasp_markers";
static const std::string DISPLAY_PATH_TOPIC_NAME = "/move_group/display_planned_path";
static const std::string CONTROLLER_JOINT_STATES_TOPIC_NAME = "/move_group/controller_joint_states";
static const std::string MONITORED_PLANNING_SCENE_TOPIC_NAME = "/move_group/monitored_planning_scene";
static const std::string MOTION_PLAN_REQUEST_TOPIC_NAME = "/move_group/motion_plan_request";

static const std::string PLANNING_SCENE_TOPIC_NAME = "/planning_scene";
static const std::string PLANNING_SCENE_WORLD_TOPIC_NAME = "/planning_scene_world";
static const std::string RECOGNIZED_OBJECT_ARRAY_TOPIC_NAME = "/recognized_object_array";
static const std::string TF_TOPIC_NAME = "/tf";
static const std::string TF_STATIC_TOPIC_NAME = "/tf_static";
static const std::string TRAJECTORY_EXECUTION_EVENT_TOPIC_NAME = "/trajectory_execution_event";

}

/// service
namespace service
{
/*
/apply_planning_scene
/check_state_validity
/clear_octomap
/compute_cartesian_path
/compute_fk
/compute_ik
/get_planning_scene
/move_group/load_map
/move_group/save_map
/plan_kinematic_path
/plan_sequence_path
/query_planner_interface
*/

static const std::string APPLY_PLANNING_SCENE_SERVICE_NAME = "/apply_planning_scene";
static const std::string STATE_VALIDITY_SERVICE_NAME = "/check_state_validity";
static const std::string CLEAR_OCTOMAP_SERVICE_NAME = "/clear_octomap";
static const std::string CARTESIAN_PATH_SERVICE_NAME = "/compute_cartesian_path";
static const std::string FK_SERVICE_NAME = "/compute_fk";
static const std::string IK_SERVICE_NAME = "/compute_ik";
static const std::string GET_PLANNER_PARAMS_SERVICE_NAME = "/get_planner_params";
static const std::string GET_PLANNING_SCENE_SERVICE_NAME = "/get_planning_scene";
static const std::string LOAD_MAP_SERVICE_NAME = "/move_group/load_map";
static const std::string SAVE_MAP_SERVICE_NAME = "/move_group/save_map";
static const std::string PLANNER_SERVICE_NAME = "/plan_kinematic_path";
static const std::string PLAN_SEQUENCE_PATH_SERVICE_NAME = "/plan_sequence_path";
static const std::string QUERY_PLANNERS_SERVICE_NAME = "/query_planner_interface";
static const std::string SET_PLANNER_PARAMS_SERVICE_NAME = "/set_planner_params";

}

/// action
namespace action
{
/*
/execute_trajectory/cancel
/execute_trajectory/feedback
/execute_trajectory/goal
/execute_trajectory/result
/execute_trajectory/status

/move_group/cancel
/move_group/feedback
/move_group/goal
/move_group/result
/move_group/status

/pickup/cancel
/pickup/feedback
/pickup/goal
/pickup/result
/pickup/status

/place/cancel
/place/feedback
/place/goal
/place/result
/place/status

/sequence_move_group/cancel
/sequence_move_group/feedback
/sequence_move_group/goal
/sequence_move_group/result
/sequence_move_group/status
*/

static const std::string EXECUTE_ACTION_NAME = "/execute_trajectory";
static const std::string MOVE_ACTION_NAME = "/move_group";
static const std::string PICKUP_ACTION_NAME = "/pickup";
static const std::string PLACE_ACTION_NAME = "/place";
static const std::string SEQUENCE_MOVE_ACTION_NAME = "/sequence_move_group";

static const std::string CONTROLLER_FOLLOW_TRAJECTORY_ACTION_NAME = "/position_joint_trajectory_controller/follow_joint_trajectory";

}

}  // namespace moveit_runtime

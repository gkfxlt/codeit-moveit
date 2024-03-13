//
// Created by fan on 2021/7/29.
//

#include "codeit/function/func_checkjs.h"

#include <codeit.hpp>
#include "codeit/model/model_moveit.h"

namespace codeit::function
{
auto CheckJS::prepareNrt(BasisFunc&, int) -> void
{
  option() |= (NOT_PRINT_EXECUTE_COUNT | NOT_LOG_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION);

  MoveitPlanner* planner = MoveitPlanner::getInstance();
  if (planner == nullptr)
  {
    return;
  }

  JointTarget jointTarget;
  for (auto& cmd_param : cmdParams())
  {
    if (cmd_param.first == "jointtarget_var")
    {
      std::string joint_target_name = std::string(cmdParams().at(cmd_param.first));
      auto& cal = this->controlSystem()->model().calculator();
      s_vec2jointtarget(
          std::any_cast<Matrix>(cal.calculateExpression("jointtarget(" + joint_target_name + ")").second).data(),
          jointTarget);
    }
  }

  move_group::JointState jointState;
  auto& move_group = planner->getMoveGroup();
  jointState.joint_names = move_group.getControllerJointNames();

  std::vector<unsigned int> motion_ids;
  if (!move_group.getMotionIdByJointName(jointState.joint_names, motion_ids))
  {
    return;
  }

  unsigned int max_index = *std::max_element(motion_ids.begin(), motion_ids.end());
  if (max_index >= sizeof(JointTarget) / sizeof(double))
  {
    return;
  }

  for (auto id : motion_ids)
  {
    jointState.positions.push_back(jointTarget.joint_vec[id]);
  }

  bool result = move_group.checkJointStateValid(jointState);
  std::cerr << "CheckJS Finished: " << result << (result ? ": valid" : ": invalid") << std::endl;
}

auto CheckJS::executeRT(BasisFunc&, int) -> int
{
  return 0;  //结束该指令
}

auto CheckJS::collectNrt(BasisFunc&, int) -> void
{
}

CheckJS::~CheckJS() = default;

CheckJS::CheckJS(const std::string& name) : BasisFunc(name)
{
  command().loadXmlStr("<Command name=\"CheckJS\">"
                       "	<GroupParam>"
                       "		<Param name=\"jointtarget_var\" default=\"j0\"/>"
                       "	</GroupParam>"
                       "</Command>");
}

CODEIT_DEFINE_BIG_FOUR_CPP(CheckJS);

}  // namespace codeit::function
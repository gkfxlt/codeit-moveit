//
// Created by fan on 2021/7/29.
//

#include "codeit/function/func_obsm.h"

#include <codeit.hpp>
#include "codeit/model/model_moveit.h"

namespace codeit::function
{
auto ObsM::prepareNrt(BasisFunc&, int) -> void
{
  option() |= (NOT_PRINT_EXECUTE_COUNT | NOT_LOG_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION);

  MoveitPlanner* planner = MoveitPlanner::getInstance();
  if (planner == nullptr)
  {
    return;
  }

  move_group::ProcessCollisionObjectParam param;

  bool clear = false;
  for (auto& cmd_param : cmdParams())
  {
    if (cmd_param.first == "operation")
    {
      param.operation = int32Param(cmd_param.first);
    }
    else if (cmd_param.first == "id")
    {
      param.id = std::string(cmdParams().at(cmd_param.first));
    }
    else if (cmd_param.first == "frame_id")
    {
      param.frame_id = std::string(cmdParams().at(cmd_param.first));
    }
    else if (cmd_param.first == "pose")
    {
      Pose pose;
      std::string robot_target_name = std::string(cmdParams().at(cmd_param.first));

      auto& cal = this->controlSystem()->model().calculator();
      s_vec2pose(std::any_cast<Matrix>(cal.calculateExpression("robottarget(" + robot_target_name + ")").second).data(),
                 pose);
      memcpy(&param.pose, &pose, sizeof(Pose));
    }
    else if (cmd_param.first == "type")
    {
      param.type = int32Param(cmd_param.first);
    }
    else if (cmd_param.first == "dimensions")
    {
      auto d = matrixParam(cmd_param.first);
      param.dimensions.assign(d.begin(), d.end());
    }
    else if (cmd_param.first == "clear")
    {
      auto val = std::string(cmdParams().at(cmd_param.first));
      clear = !((val == "false") || (val == "0"));
    }
  }

  auto& move_group = planner->getMoveGroup();
  if (clear)
    move_group.clearCollisionObject();
  else
    move_group.processCollisionObject(param);
}

auto ObsM::executeRT(BasisFunc&, int) -> int
{
  return 0;  //结束该指令
}

auto ObsM::collectNrt(BasisFunc&, int) -> void
{
}

ObsM::~ObsM() = default;

ObsM::ObsM(const std::string& name) : BasisFunc(name)
{
  command().loadXmlStr("<Command name=\"ObsM\">"
                       "	<GroupParam>"
                       "		<Param name=\"operation\" default=\"0\"/>"
                       "		<Param name=\"id\" default=\"Box_0\"/>"
                       "		<Param name=\"frame_id\" default=\"world\"/>"
                       "		<Param name=\"pose\" default=\"p0\"/>"
                       "		<Param name=\"type\" default=\"1\"/>"
                       "		<Param name=\"dimensions\" default=\"{0.2,0.2,0.2}\"/>"
                       "		<Param name=\"clear\" default=\"false\"/>"
                       "	</GroupParam>"
                       "</Command>");
}

CODEIT_DEFINE_BIG_FOUR_CPP(ObsM);

}  // namespace codeit::function
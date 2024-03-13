//
// Created by fan on 2021/7/29.
//

#include "sensor_rs_camera.hpp"

#include "codeit/function/func_perception.h"

#include <codeit.hpp>
#include "codeit/model/model_moveit.h"


namespace codeit::function
{

typedef struct
{
  enum PerceptionType
  {
    PointCloud = 0,
    DepthImage = 1,
  };
  int type;
}PerceptionParam;

auto Perception::prepareNrt(BasisFunc&, int) -> void
{
  MoveitPlanner* planner = MoveitPlannerHelper::planner();
  if (planner == nullptr)
  {
    return;
  }

  PerceptionParam param;
  for (auto& cmd_param : cmdParams())
  {
    if (cmd_param.first == "type")
    {
      param.type = int32Param(cmd_param.first);
    }
  }

   auto& cs = codeit::system::ControlSystem::instance();
   if (!cs.running())
   {
     return;
   }

   if(param.type == 0)
   {
     std::string serial_no("014122072495");
     auto it = cs.sensorPool().findByName(serial_no);
     if(it == cs.sensorPool().end())
     {
       return;
     }

     auto& sensor = *it;
     codeit::sensor::RsCameraDataParser data_parser;
     MoveitPlanner* planner = MoveitPlannerHelper::planner();

     data_parser.parseFrom(&sensor);
     pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc = data_parser.pointCloud();

     const char* point_data = (const char*)&pc->points.data()[0];
     int point_size = sizeof(pc->points.data()[0]);
     int point_num = pc->points.size();
     std::string frame_id = "camera_depth_optical_frame";
     int x_offset = (char*)&pc->points.data()[0].x - (char*)&pc->points.data()[0];
     int y_offset = (char*)&pc->points.data()[0].y - (char*)&pc->points.data()[0];
     int z_offset = (char*)&pc->points.data()[0].z - (char*)&pc->points.data()[0];
     planner->getMoveGroup().inComingPointCloud(point_data, point_size, point_num, frame_id, x_offset, y_offset, z_offset);
   }

  this->param() = param;
  for (auto& option : motorOptions())
    option |= USE_TARGET_POS | NOT_CHECK_ENABLE;
}

auto Perception::executeRT(BasisFunc&, int) -> int
{
  return 0;  //结束该指令
}

auto Perception::collectNrt(BasisFunc&, int) -> void
{
}

Perception::~Perception() = default;

Perception::Perception(const std::string& name) : BasisFunc(name)
{
  /// type=0 PointCloud2 点云  |  type=1 DeepImage 深度图
  /// freq 采集频率
  /// times=0 无限次数  |  times=1 采集一次数据

  command().loadXmlStr("<Command name=\"Perception\">"
                       "	<GroupParam>"
                       "		<Param name=\"type\" default=\"0\"/>"
                       "	</GroupParam>"
                       "</Command>");
}

CODEIT_DEFINE_BIG_FOUR_CPP(Perception);

}  // namespace codeit::function
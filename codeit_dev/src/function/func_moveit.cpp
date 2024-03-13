//
// Created by fan on 2021/7/29.
//

#include "codeit/function/func_moveit.h"

#include <codeit.hpp>
#include "codeit/model/model_moveit.h"
#include <future>
#include <queue>

/// JointTrajectoryInterpInterface
class JointTrajectoryInterpInterface
{
public:
  virtual ~JointTrajectoryInterpInterface() = default;

  virtual void set(const move_group::JointTrajectory& joint_trajectory) = 0;

  virtual std::vector<double> get(double t, bool& is_finished) = 0;

protected:
  size_t JOINT_NUM{};  //关节的数量
};

// 关节轨迹 多关节序列插值 T为插值方法(线性，Akima)
template <typename T>
class JointTrajectoryInterp : public JointTrajectoryInterpInterface
{
public:
  virtual ~JointTrajectoryInterp() = default;

  void set(const move_group::JointTrajectory& joint_trajectory)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    JOINT_NUM = joint_trajectory.joint_names.size();
    interp_.resize(JOINT_NUM);

    auto point_num = joint_trajectory.points.size();

    // time
    std::vector<double> time(point_num);
    for (size_t i = 0; i < point_num; ++i)
    {
      time[i] = joint_trajectory.points[i].time_from_start;
    }

    // val
    for (size_t i = 0; i < JOINT_NUM; ++i)
    {
      std::vector<double> val(point_num);
      for (size_t j = 0; j < point_num; ++j)
      {
        val[j] = joint_trajectory.points[j].positions[i];
      }
      interp_[i].set(point_num, time.data(), val.data());
    }
  }

  std::vector<double> get(double t, bool& is_finished)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    static std::vector<double> return_val(JOINT_NUM);
    for (size_t i = 0; i < JOINT_NUM; ++i)
    {
      return_val[i] = interp_[i].get(t, is_finished);
      if (isnan(return_val[i]))
      {
        std::cerr << "time: " << t << " pose is nan..." << std::endl;
      }
    }
    return return_val;
  }

private:
  std::mutex mutex_;
  std::vector<T> interp_;
};

// Akima 插值， 单一序列插值
class Akima
{
public:
  void set(std::size_t size, const double* t, const double* val)
  {
    std::size_t whole_size = size + 6;
    time_whole.resize(whole_size);
    pos.resize(whole_size);
    pos_p1.resize(whole_size);
    pos_p2.resize(whole_size);
    pos_p3.resize(whole_size);

    time_whole[0] = -3;
    time_whole[1] = -2;
    time_whole[2] = -1;

    pos[0] = val[0];
    pos[1] = val[0];
    pos[2] = val[0];

    for (size_t i = 0; i < size; ++i)
    {
      time_whole[3 + i] = t[i];
      pos[3 + i] = val[i];
    }

    time_whole[whole_size - 3] = t[size - 1] + 1;
    time_whole[whole_size - 2] = t[size - 1] + 2;
    time_whole[whole_size - 1] = t[size - 1] + 3;

    pos[whole_size - 3] = val[size - 1];
    pos[whole_size - 2] = val[size - 1];
    pos[whole_size - 1] = val[size - 1];

    codeit::model::s_akima(time_whole.size(), time_whole.data(), pos.data(), pos_p1.data(), pos_p2.data(),
                           pos_p3.data(), 1e-10);
  }

  double get(double t, bool& is_finished)
  {
    static const size_t offset = 3;

    is_finished = false;
    auto begin_index = offset;
    if (t <= time_whole[begin_index])
    {
      return pos[begin_index];
    }

    auto end_index = time_whole.size() - 1 - offset;
    if (t >= time_whole[end_index])
    {
      is_finished = true;
      return pos[end_index];
    }

    return codeit::model::s_akima_at(time_whole.size(), time_whole.data(), pos.data(), pos_p1.data(), pos_p2.data(),
                                     pos_p3.data(), t);
  }

private:
  std::vector<double> time_whole;
  std::vector<double> pos;
  std::vector<double> pos_p1, pos_p2, pos_p3;
};
// Akima多关节序列插值--轨迹
using AkimaJointTrajectoryInterp = JointTrajectoryInterp<Akima>;

// Linear 插值， 单一序列插值
class Linear
{
public:
  void set(std::size_t size, const double* t, const double* val)
  {
    time_whole.resize(size);
    pos.resize(size);

    memcpy(time_whole.data(), t, sizeof(double) * size);
    memcpy(pos.data(), val, sizeof(double) * size);
  }

  double get(double t, bool& is_finished)
  {
    static const size_t offset = 0;

    is_finished = false;
    auto begin_index = offset;
    if (t <= time_whole[begin_index])
    {
      return pos[begin_index];
    }

    auto end_index = time_whole.size() - 1 - offset;
    if (t >= time_whole[end_index])
    {
      is_finished = true;
      return pos[end_index];
    }

    size_t index = 0;
    for (size_t i = 0; i < time_whole.size(); ++i)
    {
      if (time_whole[i] > t)
      {
        index = i;
        break;
      }
    }

    double t0 = time_whole[index - 1];
    double t1 = time_whole[index];
    double p0 = pos[index - 1];
    double p1 = pos[index];
    double alpha = (t - t0) / (t1 - t0);
    return p0 + alpha * (p1 - p0);
  }

private:
  std::vector<double> time_whole;
  std::vector<double> pos;
};
// 线性多关节序列插值--轨迹
using LinearJointTrajectoryInterp = JointTrajectoryInterp<Linear>;

/// JointTrajectoryInterpInterface end

/// 轨迹监控，计算时间点状态，发布虚拟状态。
class TrajectoryMonitor
{
  using Trajectory = move_group::JointTrajectory;

public:
  explicit TrajectoryMonitor(MoveitPlanner* planner)
  {
    planner_ = planner;
    max_motion_id_ = 0;

    // for trajectory
    is_current_trajectory_finished_ = false;
    current_trajectory_time_ = 0;

    // for pause
    pause_trajectory_time_ = std::numeric_limits<double>::max();

    // for planning
    is_planning_ = false;
    is_plan_finished_ = false;
    stop_screen_update_handle_thread_ = false;
    screen_update_handle_thread_ = std::thread([this]() {
      std::cerr << "start monitor thread ..." << std::endl;
      while (!stop_screen_update_handle_thread_)
      {
        {
          std::unique_lock<std::mutex> locker(mutex_);
          condition_.wait(locker);  // Unlock _mutex and wait to be notified
        }
        while(is_screen_update)
        {
          is_screen_update = false;
          dealScreenUpdate();
        }
      }
    });

    joint_trajectory_interp_ = std::make_shared<AkimaJointTrajectoryInterp>();
    planner_->getMoveGroup().registerScreenUpdateCallback([this] {
      std::unique_lock<std::mutex> locker(mutex_);
      is_screen_update = true;
      condition_.notify_one();
    });
  }

  ~TrajectoryMonitor()
  {
    planner_->getMoveGroup().registerScreenUpdateCallback(nullptr);
    stop_screen_update_handle_thread_ = true;
    condition_.notify_one();
    if (screen_update_handle_thread_.joinable())
    {
      screen_update_handle_thread_.join();
    }
  }

  move_group::PlanIt& getPlanData()
  {
    return plan_data_;
  }

  void dealScreenUpdate()
  {
    std::cerr << "dealScreenUpdate()..." << std::endl;

    // 计算 current_trajectory_index_
    int current_index = trajectory_.points.size() - 1;
    for (int i = 0; i < trajectory_.points.size(); ++i)
    {
      if (trajectory_.points[i].time_from_start > current_trajectory_time_)
      {
        current_index = i;
        break;
      }
    }

    // 计算碰撞点
    int collision_index;
    if (!planner_->getMoveGroup().checkCollision(plan_data_.request.group_name, trajectory_, current_index,
                                                 collision_index))
    {
      std::cerr << "dealScreenUpdate()... finished.    no collision..." << std::endl;
      return;
    }

    std::cerr << "collision index:" << collision_index << " current index:" << current_index << std::endl;

    // 碰撞点距离较远，时间超过0.5S
    if (trajectory_.points[collision_index].time_from_start - trajectory_.points[current_index].time_from_start >= 0.5)
    {
      for (int i = current_index; i <= collision_index; ++i)
      {
        if (trajectory_.points[i].time_from_start - trajectory_.points[current_index].time_from_start >= 0.5)
        {
          std::cerr << "collision time >= 0.5s.  replan start index:" << i << std::endl;

          trajectory_.points.resize(i + 1);
          pause_trajectory_point_ = trajectory_.points[i];
          break;
        }
      }
    }
    else  // 碰撞点距离较近，时间不到0.5S
    {
      std::cerr << "collision time < 0.5s.  stop and replan." << std::endl;

      trajectory_.points.resize(current_index + 1);
      pause_trajectory_point_ = trajectory_.points[current_index];
    }

    // rePlan
    is_planning_ = true;
    is_plan_finished_ = false;
    pause_trajectory_time_ = pause_trajectory_point_.time_from_start;
    plan_data_.request.joint_start.joint_names = trajectory_.joint_names;
    plan_data_.request.joint_start.positions = pause_trajectory_point_.positions;
    bool flag = planner_->getMoveGroup().planIt(plan_data_.request, plan_data_.response);
    if (flag)
    {
      std::cerr << "planIt... finished.   trajectory_finished: " << is_current_trajectory_finished_ << std::endl;

      auto temp = pause_trajectory_point_;
      for (int i = 0; i < 3; ++i)
      {
        temp.time_from_start += 0.0025;
        trajectory_.points.push_back(temp);
      }

      // build trajectory_
      double time_offset = pause_trajectory_point_.time_from_start + 0.01;  // 10ms
      for (auto& jt : plan_data_.response.joint_trajectory.points)
      {
        jt.time_from_start += time_offset;
        trajectory_.points.push_back(jt);
      }
    }

    is_plan_finished_ = true;
    std::cerr << "dealScreenUpdate()... finished." << std::endl;
  }

  bool initTrajectory(const Trajectory& trajectory)
  {
    if (planner_->getMoveGroup().getMotionIdByJointName(trajectory.joint_names, motion_ids_))
    {
      max_motion_id_ = *std::max_element(motion_ids_.begin(), motion_ids_.end());
      trajectory_ = trajectory;
      joint_trajectory_interp_->set(trajectory_);
      return true;
    }
    return false;
  }

  unsigned int getMaxMotionId() const
  {
    return max_motion_id_;
  }

  std::vector<unsigned int>& getMotionIds()
  {
    return motion_ids_;
  }

  std::vector<double> calcTimePositions(double t)
  {
    static std::vector<double> positions;

    current_trajectory_time_ = t;

    if (t < pause_trajectory_time_)
    {
      bool is_finished = false;
      positions = joint_trajectory_interp_->get(t, is_finished);
      is_current_trajectory_finished_ = is_finished;
    }
    else
    {
      is_current_trajectory_finished_ = true;

      if (is_planning_ && is_plan_finished_)
      {
        is_planning_ = false;
        is_plan_finished_ = false;
        is_current_trajectory_finished_ = false;
        joint_trajectory_interp_->set(trajectory_);
        pause_trajectory_time_ = std::numeric_limits<double>::max();
      }
    }
    return positions;
  }

  void execTrajectoryVirtual(double t)
  {
    static move_group::JointState jointState;
    jointState.joint_names = trajectory_.joint_names;
    jointState.positions = calcTimePositions(t);
    planner_->getMoveGroup().publishJointState(jointState);
  }

  bool isFinished()
  {
    bool is_finished = (!is_planning_) && is_current_trajectory_finished_;
    if (is_finished)
    {
      planner_->getMoveGroup().registerScreenUpdateCallback(nullptr);
    }
    return is_finished;
  }

  void addTestCollision()
  {
    move_group::ProcessCollisionObjectParam param;
    param.operation = 0;
    param.id = "test1";
    param.frame_id = "world";
    param.pose.x = 0.5;
    param.pose.y = 0;
    param.pose.z = 0.35;
    param.pose.q3 = 1;
    param.type = 1;
    param.dimensions = { 0.1, 0.1, 0.05 };
    planner_->getMoveGroup().processCollisionObject(param);

    param.id = "test2";
    param.frame_id = "world";
    param.pose.x = 0.3;
    param.pose.y = 0.3;
    param.pose.z = 0.42;
    planner_->getMoveGroup().processCollisionObject(param);
  }

private:
  // planner
  MoveitPlanner* planner_;
  std::vector<unsigned int> motion_ids_;
  unsigned int max_motion_id_{};

  // trajectory
  Trajectory trajectory_;
  atomic_bool is_current_trajectory_finished_;
  double current_trajectory_time_;

  // for pause
  double pause_trajectory_time_;
  move_group::JointTrajectoryPoint pause_trajectory_point_;

  // for planning
  atomic_bool is_planning_;
  atomic_bool is_plan_finished_;
  move_group::PlanIt plan_data_;

  // for screen_update_handle_thread_
  std::mutex mutex_;
  atomic_bool is_screen_update;
  std::condition_variable condition_;
  bool stop_screen_update_handle_thread_;
  std::thread screen_update_handle_thread_;

  // JointTrajectoryInterp
  std::shared_ptr<JointTrajectoryInterpInterface> joint_trajectory_interp_;
};

namespace codeit::function
{
struct MoveItLocalParam
{
  double t;
  int exec;  // 0: 只规划不执行；1:规划并真实执行。2:规划并虚拟执行
  int show;  // show==0时不起作用；show=1显示当前关节TF，show=2显示当前连杆末端坐标。默认值：0
             // group_name有效时显示组内关节和连杆，无效时显示所有关节和连杆。
             // show != 0 时不会规划运行；
  std::shared_ptr<TrajectoryMonitor> trajectory_monitor;
};

auto MoveIt::prepareNrt(BasisFunc&, int) -> void
{
  MoveitPlanner* planner = MoveitPlanner::getInstance();
  if (planner == nullptr)
  {
    option() |= (NOT_PRINT_EXECUTE_COUNT | NOT_LOG_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION);
    return;
  }

  MoveItLocalParam param;
  param.t = 0;
  param.exec = 1;
  param.trajectory_monitor = std::make_shared<TrajectoryMonitor>(planner);
  auto& plan_data = param.trajectory_monitor->getPlanData();
  auto& request = plan_data.request;

  for (auto& cmd_param : cmdParams())
  {
    if (cmd_param.first == "robottarget_var")
    {
      vector<std::string> robot_target_names;
      split(std::string(cmdParams().at(cmd_param.first)), robot_target_names);

      for (auto& robot_target_name : robot_target_names)
      {
        RobotTarget robotTarget;
        auto& cal = this->controlSystem()->model().calculator();
        s_vec2robtarget(
            std::any_cast<Matrix>(cal.calculateExpression("robottarget(" + robot_target_name + ")").second).data(),
            robotTarget);

        request.waypoints.emplace_back();
        memcpy(&request.waypoints.back(), &robotTarget, sizeof(robotTarget));
      }
      request.robot_target = request.waypoints.back();
    }
    else if (cmd_param.first == "pipeline_id")
    {
      request.pipeline_id = std::string(cmdParams().at(cmd_param.first));
    }
    else if (cmd_param.first == "group_name")
    {
      request.group_name = std::string(cmdParams().at(cmd_param.first));
    }
    else if (cmd_param.first == "allowed_planning_time")
    {
      request.allowed_planning_time = doubleParam(cmd_param.first);
    }
    else if (cmd_param.first == "exec")
    {
      param.exec = int32Param(cmd_param.first);
    }
    else if (cmd_param.first == "show")
    {
      param.show = int32Param(cmd_param.first);
    }
  }

  if (param.show == 1)
  {
    std::map<std::string, move_group::Pose> poses;
    planner->getMoveGroup().getGroupJointTransforms(request.group_name, poses);
    for (auto& [name, p] : poses)
    {
      std::cout << "name:\"" << name << "\" "
                << "pose:{" << p.x << "," << p.y << "," << p.z << "," << p.q1 << "," << p.q2 << "," << p.q3 << ","
                << p.q4 << "}" << std::endl;
    }
    option() |= (NOT_PRINT_EXECUTE_COUNT | NOT_LOG_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION);
    return;
  }
  else if (param.show == 2)
  {
    std::map<std::string, move_group::Pose> poses;
    planner->getMoveGroup().getGroupGlobalLinkTransforms(request.group_name, poses);
    for (auto& [name, p] : poses)
    {
      std::cout << "name:\"" << name << "\" "
                << "pose:{" << p.x << "," << p.y << "," << p.z << "," << p.q1 << "," << p.q2 << "," << p.q3 << ","
                << p.q4 << "}" << std::endl;
    }
    option() |= (NOT_PRINT_EXECUTE_COUNT | NOT_LOG_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION);
    return;
  }

  if (param.exec == 1)
  {
    planner->getCurrentJointState(plan_data.request.joint_start);
  }

  if (!planner->getMoveGroup().planIt(plan_data.request, plan_data.response))
  {
    std::cerr << "** planIt failed." << std::endl;
    option() |= (NOT_PRINT_EXECUTE_COUNT | NOT_LOG_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION);
    return;
  }

  if (param.exec == 0)
  {
    std::cerr << "** planIt succeed. plan only." << std::endl;
    option() |= (NOT_PRINT_EXECUTE_COUNT | NOT_LOG_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION);
    return;
  }

  if(!param.trajectory_monitor->initTrajectory(plan_data.response.joint_trajectory))
  {
    std::cerr << "** initTrajectory failed. joint names match motion id error." << std::endl;
    option() |= (NOT_PRINT_EXECUTE_COUNT | NOT_LOG_CMD_INFO | NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION);
    return;
  }

  std::cerr << "** planIt succeed. start run." << std::endl;

  this->param() = param;

  for (auto& option : motorOptions())
    option |= USE_TARGET_POS | NOT_CHECK_ENABLE;
}

auto MoveIt::executeRT(BasisFunc&, int) -> int
{
  if (!this->param().has_value())
  {
    return -1;
  }

  auto& param = std::any_cast<MoveItLocalParam&>(this->param());

  // for test
  //  if (count() == 400)
  //  {
  //    param.trajectory_monitor->addTestCollision();
  //  }

  double ut = controlSystem()->ut(cmdSubId());
  double utBreak = controlSystem()->utBreakRt(cmdSubId());
  double dt = controller()->samplePeriodNs() / 1.0e9 * ut * utBreak;
  param.t += dt;

  if (param.exec == 1)
  {
    auto& motion_ids = param.trajectory_monitor->getMotionIds();
    auto max_motion_id = param.trajectory_monitor->getMaxMotionId();
    auto positions = param.trajectory_monitor->calcTimePositions(param.t);

    auto num = std::min(controller()->motionPool().size(), model()->motionPool().size());
    if (num > max_motion_id)
    {
      for (int i = 0; i < motion_ids.size(); ++i)
      {
        model()->motionPool().at(motion_ids[i]).setMp(positions[i]);
      }
    }
    else
    {
      std::cerr << "motion num is not enough for use." << std::endl;
      return -1;
    }
  }
  else
  {
    param.trajectory_monitor->execTrajectoryVirtual(param.t);
  }

  if (param.trajectory_monitor->isFinished())
    return 0;  //结束该指令

  return 1;
}

auto MoveIt::collectNrt(BasisFunc&, int) -> void
{
}

MoveIt::~MoveIt() = default;

MoveIt::MoveIt(const std::string& name) : BasisFunc(name)
{
  command().loadXmlStr("<Command name=\"MoveIt\">"
                       "	<GroupParam>"
                       "		<Param name=\"robottarget_var\" default=\"p0\"/>"
                       "		<Param name=\"pipeline_id\" default=\"ompl\"/>"
                       "		<Param name=\"group_name\" default=\"manipulator\"/>"
                       "		<Param name=\"allowed_planning_time\" default=\"5.0\"/>"
                       "		<Param name=\"exec\" default=\"1\"/>"
                       "		<Param name=\"show\" default=\"0\"/>"
                       "	</GroupParam>"
                       "</Command>");
}

CODEIT_DEFINE_BIG_FOUR_CPP(MoveIt);

}  // namespace codeit::function

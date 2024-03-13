#include "codeit/function/func_movejs.hpp"
#include <codeit.hpp>

namespace codeit::function
{
struct MoveJSParam
{
  std::int32_t motion_id;
  double amp, freq;
  vector<double> axis_begin_pos_vec;
  double time{ 0 };
};

auto MoveJS::prepareNrt(BasisFunc&, int) -> void
{
  MoveJSParam param;
  for (auto cmd_param : cmdParams())
  {
    if (cmd_param.first == "motion_id")
    {
      param.motion_id = int32Param(cmd_param.first);
    }
    if (cmd_param.first == "amp")
      param.amp = doubleParam(cmd_param.first);
    if (cmd_param.first == "freq")
      param.freq = doubleParam(cmd_param.first);
  }

  auto num = controller()->motionPool().size();
  param.axis_begin_pos_vec.resize(num, 0);

  auto& cout = controller()->mout();  // cout区别于std::cout
  cout << num << std::endl;

  this->param() = param;
  for (auto& option : motorOptions())
    option |= USE_TARGET_POS | NOT_CHECK_ENABLE;

  // auto& cs = codeit::system::ControlSystem::instance();
  // cs.interfacePool().at(0).name();
  // cs.sensorPool().at(0).name();
}

auto MoveJS::executeRT(BasisFunc&, int) -> int
{
  auto& param = std::any_cast<MoveJSParam&>(this->param());
  if (count() == 1)
  {
    double a = 1;
  }
  //////*********************** MoveSine 规划 *************************//
  double dt = controller()->samplePeriodNs() / 1.0e9;

  auto running_flag = true;
  param.time += dt;
  double pos = param.amp * sin(2 * PI * param.freq * param.time);

  modelPool()->at(0);
  ioModelPool();
  //	向模型输出指令角度	//
  for (int32_t i = 0; i < 6; i++)
  {
    model()->motionPool()[i].setMp(pos);
  }
  // model()->motionPool()[param.motion_id].setMp(pos);
  if (model()->solverPool().at(1).kinPos())
  {
    return -1;
  }

  // 打印 //
  auto& cout = controller()->mout();  // cout区别于std::cout
  cout << "target_count:	" << count() << "	"
       << "pos:	" << pos << std::endl
       << "mp:	" << model()->motionPool()[0].mp() << std::endl
       << "actualPos:	" << controller()->motionPool().at(0).actualPos() << std::endl
       << "targetPos:	" << controller()->motionPool().at(0).targetPos() << std::endl;

  // 保存 //
  auto& lout = controller()->lout();
  lout << pos << endl;  //保存pos信息至txt文件

  if (count() == 1000)
  {
    model()->motionPool()[0].setMp(0);
    model()->motionPool()[1].setMp(-1.5);
    model()->motionPool()[2].setMp(1.5);
    model()->motionPool()[3].setMp(0);
    model()->motionPool()[4].setMp(0);
    model()->motionPool()[5].setMp(0);

    return 0;  //结束该指令
  }

  return 1;    //仍在运行该指令
}

auto MoveJS::collectNrt(BasisFunc&, int) -> void
{
}

MoveJS::~MoveJS() = default;

MoveJS::MoveJS(const std::string& name) : BasisFunc(name)
{
  command().loadXmlStr("<Command name=\"MoveJS\">"
                       "	<GroupParam>"
                       "		<Param name=\"amp\" default=\"0.5\"/>"
                       "		<Param name=\"freq\" default=\"1\"/>"
                       "		<Param name=\"motion_id\" default=\"0\" abbreviation=\"m\"/>"
                       "	</GroupParam>"
                       "</Command>");
}

CODEIT_DEFINE_BIG_FOUR_CPP(MoveJS);

}  // namespace codeit::function
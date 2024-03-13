//
// Created by yjh on 6/28/21.
//

#include "param_adapter.h"
#include "fake_node_handle/fake_node_handle.h"
#include "yaml-cpp/yaml.h"

template <typename T>
void setParam(const ros::NodeHandle* pNode, const std::string& name, std::any& val)
{
  pNode->setParam(name, std::any_cast<T>(val));
}

using setParamFunc = void (*)(const ros::NodeHandle*, const std::string&, std::any&);

void parseAndSetParam(const ros::NodeHandle* pNode, const std::string& name,
                      const std::shared_ptr<_ros::ParamValueOptions>& val_ops)
{
  static std::map<std::string, setParamFunc> setParamMap = {
    { typeid(bool).name(), setParam<bool> },
    { typeid(int).name(), setParam<int> },
    { typeid(double).name(), setParam<double> },
    { typeid(std::string).name(), setParam<std::string> },
    { typeid(std::vector<bool>).name(), setParam<std::vector<bool>> },
    { typeid(std::vector<int>).name(), setParam<std::vector<int>> },
    { typeid(std::vector<double>).name(), setParam<std::vector<double>> },
    { typeid(std::vector<std::string>).name(), setParam<std::vector<std::string>> },
  };

  if (setParamMap.find(val_ops->type_) != setParamMap.end())
  {
    setParamMap[val_ops->type_](pNode, name, val_ops->val_);
  }
  else if (val_ops->type_ != typeid(YAML::Node).name())
  {
    std::cerr << "ParamAdapter::parseAndSetParam type not set.  name:" << name << "  type:" << val_ops->type_
              << std::endl;
  }
}

void ParamAdapter::init(ros::NodeHandle* pNode)
{
  _ros::NodeHandle nh;
  auto params = nh.getAllParam();

  for (auto& param : params)
  {
    parseAndSetParam(pNode, param.first, param.second);
  }
}

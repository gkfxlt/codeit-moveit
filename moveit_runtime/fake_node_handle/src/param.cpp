//
// Created by Administrator on 2021/6/16.
//
#include "fake_node_handle/param.h"

using namespace _ros;
ParamCenter* ParamCenter::Instance()
{
  static ParamCenter instance;
  return &instance;
}

void ParamCenter::setParam(const std::string& param, std::shared_ptr<ParamValueOptions>& ops)
{
  all_params_[param] = ops;
}

bool ParamCenter::getParam(const std::string& param, std::shared_ptr<ParamValueOptions>& ops)
{
  if (all_params_.find(param) == all_params_.end())
  {
    return false;
  }
  ops = all_params_[param];
  return true;
}

bool ParamCenter::hasParam(const std::string& param)
{
  if (all_params_.find(param) == all_params_.end())
  {
    return false;
  }
  return true;
}

bool ParamCenter::searchParam(const std::string& param_name, std::string& full_param_name)
{
  for (const auto& path : all_params_)
  {
    if (path.first.find(param_name) != std::basic_string<char>::npos)
    {
      full_param_name = path.first;
      return true;
    }
  }
  return false;
}

void ParamCenter::clearAll()
{
  all_params_.clear();
}

std::map<std::string, std::shared_ptr<ParamValueOptions>>& ParamCenter::getAllParam()
{
    return all_params_;
}

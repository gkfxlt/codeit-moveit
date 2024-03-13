//
// Created by Administrator on 2021/6/7.
//

#pragma once

#include <string>
#include <any>
#include <map>
#include <memory>

namespace _ros
{

struct ParamValueOptions
{
  template <class T>
  void init(const std::string& name, T val)
  {
    name_ = name;
    type_ = typeid(T).name();
    val_ = val;
  }

  std::string name_;
  std::string type_;
  std::any val_;
};

class ParamCenter
{
public:
  static ParamCenter* Instance();

  void setParam(const std::string& param, std::shared_ptr<ParamValueOptions>& ops);

  bool getParam(const std::string& param, std::shared_ptr<ParamValueOptions>& ops);

  bool hasParam(const std::string& param);

  bool searchParam(const std::string& param_name, std::string& full_param_name);

  void clearAll();

  std::map<std::string, std::shared_ptr<ParamValueOptions>>& getAllParam();

private:
  ParamCenter() = default;
  std::map<std::string, std::shared_ptr<ParamValueOptions>> all_params_;
};

}  // namespace _ros

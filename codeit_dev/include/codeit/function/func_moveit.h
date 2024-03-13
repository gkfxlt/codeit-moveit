//
// Created by fan on 2021/7/29.
//

#ifndef CODEIT_MOVEIT_FUNC_MOVEIT_H
#define CODEIT_MOVEIT_FUNC_MOVEIT_H

#include <codeit/function/basefunc.hpp>

namespace codeit::function
{
class MoveIt : public BasisFunc
{
public:
  auto virtual prepareNrt(BasisFunc&, int) -> void;
  auto virtual executeRT(BasisFunc&, int) -> int;
  auto virtual collectNrt(BasisFunc&, int) -> void;

  virtual ~MoveIt();
  explicit MoveIt(const std::string& name = "moveit");
  CODEIT_REGISTER_TYPE(MoveIt);
  CODEIT_DECLARE_BIG_FOUR(MoveIt);
};
}  // namespace codeit::function

#endif  // CODEIT_MOVEIT_FUNC_MOVEIT_H

//
// Created by fan on 2021/7/29.
//

#ifndef CODEIT_FUNC_CHECKJS_H
#define CODEIT_FUNC_CHECKJS_H

#include <codeit/function/basefunc.hpp>

namespace codeit::function
{
class CheckJS : public BasisFunc
{
public:
  auto virtual prepareNrt(BasisFunc&, int) -> void;
  auto virtual executeRT(BasisFunc&, int) -> int;
  auto virtual collectNrt(BasisFunc&, int) -> void;

  virtual ~CheckJS();
  explicit CheckJS(const std::string& name = "CheckJS");
  CODEIT_REGISTER_TYPE(CheckJS);
  CODEIT_DECLARE_BIG_FOUR(CheckJS);
};
}  // namespace codeit::function

#endif  // CODEIT_FUNC_CHECKJS_H

//
// Created by fan on 2021/7/29.
//

#ifndef CODEIT_FUNC_OBSM_H
#define CODEIT_FUNC_OBSM_H

#include <codeit/function/basefunc.hpp>

namespace codeit::function
{
class ObsM : public BasisFunc
{
public:
  auto virtual prepareNrt(BasisFunc&, int) -> void;
  auto virtual executeRT(BasisFunc&, int) -> int;
  auto virtual collectNrt(BasisFunc&, int) -> void;

  virtual ~ObsM();
  explicit ObsM(const std::string& name = "ObsM");
  CODEIT_REGISTER_TYPE(ObsM);
  CODEIT_DECLARE_BIG_FOUR(ObsM);
};
}  // namespace codeit::function

#endif  // CODEIT_FUNC_OBSM_H

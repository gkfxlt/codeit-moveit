//
// Created by fan on 2021/7/29.
//

#ifndef CODEIT_FUNC_PERCEPTION_H
#define CODEIT_FUNC_PERCEPTION_H

#include <codeit/function/basefunc.hpp>

namespace codeit::function
{
class Perception : public BasisFunc
{
public:
  auto virtual prepareNrt(BasisFunc&, int) -> void;
  auto virtual executeRT(BasisFunc&, int) -> int;
  auto virtual collectNrt(BasisFunc&, int) -> void;

  virtual ~Perception();
  explicit Perception(const std::string& name = "Perception");
  CODEIT_REGISTER_TYPE(Perception);
  CODEIT_DECLARE_BIG_FOUR(Perception);
};
}  // namespace codeit::function

#endif  // CODEIT_FUNC_OBSM_H

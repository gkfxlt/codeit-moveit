#ifndef FUNC_MOVEJS_H_
#define FUNC_MOVEJS_H_

#include <codeit/function/basefunc.hpp>

namespace codeit::function
{
class MoveJS : public BasisFunc
{
public:
  auto virtual prepareNrt(BasisFunc&, int) -> void;
  auto virtual executeRT(BasisFunc&, int) -> int;
  auto virtual collectNrt(BasisFunc&, int) -> void;

  virtual ~MoveJS();
  explicit MoveJS(const std::string& name = "myplan");
  CODEIT_REGISTER_TYPE(MoveJS);
  CODEIT_DECLARE_BIG_FOUR(MoveJS);
};
}  // namespace codeit::function

#endif
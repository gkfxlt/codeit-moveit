#ifndef PLUGINS1_H
#define PLUGINS1_H

#include "base.hpp"
#include <iostream>

#ifdef WIN32
#define WIN_EXPORT __declspec(dllexport)
#else
#define WIN_EXPORT
#endif

namespace animal
{
class WIN_EXPORT Dog : public Base
{
public:
  virtual void saySomething()
  {
    std::cout << "Bark" << std::endl;
  }
};

class WIN_EXPORT Cat : public Base
{
public:
  virtual void saySomething()
  {
    std::cout << "Meow" << std::endl;
  }
};

class WIN_EXPORT Duck : public Base
{
public:
  virtual void saySomething()
  {
    std::cout << "Quack" << std::endl;
  }
};

class WIN_EXPORT Cow : public Base
{
public:
  virtual void saySomething()
  {
    std::cout << "Moooo" << std::endl;
  }
};

class WIN_EXPORT Sheep : public Base
{
public:
  virtual void saySomething()
  {
    std::cout << "Baaah" << std::endl;
  }
};
}  // namespace animal

#endif  // PLUGINS1_H

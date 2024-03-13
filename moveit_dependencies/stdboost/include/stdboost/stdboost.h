
#pragma once

#include <map>
#include <functional>
#include <locale>

namespace stdboost
{
/// noncopyable
class noncopyable
{
public:
  noncopyable(const noncopyable&) = delete;
  const noncopyable& operator=(const noncopyable&) = delete;

protected:
  noncopyable() = default;
  ~noncopyable() = default;
};

/// signals2::connection  and  signals2::signal
namespace signals2
{
class connection
{
public:
  connection()
  {
    connected_ = false;
  }

  explicit connection(const std::function<void()>& f)
  {
    disconnect_func_ = f;
    connected_ = true;
  }

  bool connected() const
  {
    return connected_;
  }

  void disconnect()
  {
    disconnect_func_();
    connected_ = false;
  }

private:
  bool connected_;
  std::function<void()> disconnect_func_;
};

template <typename Func>
class signal
{
public:
  connection connect(std::function<Func>&& f)
  {
    return assgin(f);
  }
  connection connect(const std::function<Func>& f)
  {
    return assgin(f);
  }
  void disconnect(int key)
  {
    connections_.erase(key);
  }

  template <typename... Args>
  void operator()(Args&&... args)
  {
    for (auto& it : connections_)
    {
      it.second(std::forward<Args>(args)...);
    }
  }

private:
  template <typename F>
  connection assgin(F&& f)
  {
    int k = observer_id_++;
    connections_.emplace(k, std::forward<F>(f));
    return connection([this, k]() { disconnect(k); });
  }
  int observer_id_;
  std::map<int, std::function<Func>> connections_;
};
}  // namespace signals2

/// io::ios_all_saver
namespace io
{
template <typename Ch, class Tr = ::std::char_traits<Ch>>
class basic_ios_all_saver
{
public:
  typedef ::std::basic_ios<Ch, Tr> state_type;

  explicit basic_ios_all_saver(state_type& s)
    : s_save_(s)
    , a1_save_(s.flags())
    , a2_save_(s.precision())
    , a3_save_(s.width())
    , a4_save_(s.rdstate())
    , a5_save_(s.exceptions())
    , a6_save_(s.tie())
    , a7_save_(s.rdbuf())
    , a8_save_(s.fill())
    , a9_save_(s.getloc())
  {
  }

  ~basic_ios_all_saver()
  {
    this->restore();
  }

  void restore()
  {
    s_save_.imbue(a9_save_);
    s_save_.fill(a8_save_);
    s_save_.rdbuf(a7_save_);
    s_save_.tie(a6_save_);
    s_save_.exceptions(a5_save_);
    s_save_.clear(a4_save_);
    s_save_.width(a3_save_);
    s_save_.precision(a2_save_);
    s_save_.flags(a1_save_);
  }

private:
  state_type& s_save_;
  typename state_type::fmtflags const a1_save_;
  ::std::streamsize const a2_save_;
  ::std::streamsize const a3_save_;
  typename state_type::iostate const a4_save_;
  typename state_type::iostate const a5_save_;
  ::std::basic_ostream<Ch, Tr>* const a6_save_;
  ::std::basic_streambuf<Ch, Tr>* const a7_save_;
  typename state_type::char_type const a8_save_;
  ::std::locale const a9_save_;

  basic_ios_all_saver& operator=(const basic_ios_all_saver&);
};

typedef basic_ios_all_saver<char> ios_all_saver;
}  // namespace io

}  // namespace stdboost

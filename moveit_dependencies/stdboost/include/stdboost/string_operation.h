
#pragma once

#include <string>
#include <regex>

namespace stdboost
{
inline std::vector<std::string> split(const std::string& s, const std::string& seprate, bool remove_null = true)
{
  std::vector<std::string> ret;
  int seprate_len = seprate.length();
  int start = 0;
  int index;
  while ((index = s.find(seprate, start)) != -1)
  {
    std::string temp = s.substr(start, index - start);
    if (!remove_null || !temp.empty())
    {
      ret.push_back(temp);
    }
    start = index + seprate_len;
  }
  if (start < s.length())
    ret.push_back(s.substr(start));
  return ret;
}

inline std::vector<std::string> resplit(const std::string& s, std::string rgx_str = "\\s+", bool remove_null = true)
{
  std::vector<std::string> elems;
  std::regex rgx(rgx_str);

  std::sregex_token_iterator iter(s.begin(), s.end(), rgx, -1);
  std::sregex_token_iterator end;

  while (iter != end)
  {
    if (!remove_null || (*iter).length())
    {
      elems.push_back(*iter);
    }
    ++iter;
  }

  return elems;
}

inline std::string& ltrim(std::string& s)
{
  if (s.empty())
    return s;
  std::string::const_iterator iter = s.begin();
  while (iter != s.end() && isspace(*iter++))
    ;
  s.erase(s.begin(), --iter);
  return s;
}

inline std::string& rtrim(std::string& s)
{
  if (s.empty())
    return s;
  std::string::const_iterator iter = s.end();
  while (iter != s.begin() && isspace(*--iter))
    ;
  s.erase(++iter, s.end());
  return s;
}

inline std::string& trim(std::string& s)
{
  ltrim(s);
  rtrim(s);
  return s;
}

inline std::string trim_copy(std::string s)
{
  return trim(s);
}

inline bool starts_with(const std::string& str, const std::string& prefix)
{
  return prefix.size() <= str.size() && std::equal(prefix.cbegin(), prefix.cend(), str.cbegin());
}

inline bool ends_with(const std::string& str, const std::string& suffix)
{
  return suffix.size() <= str.size() && std::equal(suffix.crbegin(), suffix.crend(), str.crbegin());
}

inline std::string::size_type index_of(const std::string& str, const std::string& substr)
{
  return str.find(substr);
}

inline std::string to_upper(const std::string& str)
{
  std::string upper(str.size(), '\0');
  std::transform(str.cbegin(), str.cend(), upper.begin(), ::toupper);
  return upper;
}

inline std::string to_lower(const std::string& str)
{
  std::string lower(str.size(), '\0');
  std::transform(str.cbegin(), str.cend(), lower.begin(), ::tolower);
  return lower;
}

inline void replace_str_once(std::string& base_str, const std::string& find_str, const std::string& replace_str)
{
  auto pos = base_str.find(find_str);
  if (pos != -1)
  {
    base_str.replace(pos, find_str.length(), replace_str);
  }
}

inline void replace_all(std::string& base_str, const std::string& find_str, const std::string& replace_str)
{
  auto pos = base_str.find(find_str);
  auto t_size = find_str.size();
  auto r_size = replace_str.size();

  while (pos != std::string::npos)
  {
    base_str.replace(pos, t_size, replace_str);
    pos = base_str.find(find_str, pos + r_size);
  }
}

//#include <stdarg.h>
//
//inline std::string format(const char* fmt, ...)
//{
//  va_list ap;
//  va_start(ap, fmt);
//  int len = vsnprintf(nullptr, 0, fmt, ap);
//  va_end(ap);
//  std::string buf(len + 1, '\0');
//  va_start(ap, fmt);
//  vsnprintf(&buf[0], buf.size(), fmt, ap);
//  va_end(ap);
//  buf.pop_back();
//  return buf;
//}

}  // namespace stdboost


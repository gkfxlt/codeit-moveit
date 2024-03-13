//
// Created by Administrator on 2021/6/10.
//

#pragma once

#include <string>
#include <fstream>
#include <filesystem>

inline bool loadFileToString(const std::string& path, std::string& buffer)
{
  if (path.empty())
  {
    return false;
  }

  if (!std::filesystem::exists(path))
  {
    return false;
  }

  std::ifstream stream(path.c_str());
  if (!stream.good())
  {
    return false;
  }

  // Load the file to a string using an efficient memory allocation technique
  stream.seekg(0, std::ios::end);
  buffer.reserve(stream.tellg());
  stream.seekg(0, std::ios::beg);
  buffer.assign((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  stream.close();

  return true;
}

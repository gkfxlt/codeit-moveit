/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "plugin_loader/plugin_loader.hpp"
#include <filesystem>


//#include <sys/stat.h>
//#include <string>
//
//inline bool is_file_exists(const std::string& name)
//{
//  struct stat buffer;
//  return (stat(name.c_str(), &buffer) == 0);
//}

namespace plugin_loader
{
bool PluginLoader::has_unmananged_instance_been_created_ = false;

bool PluginLoader::hasUnmanagedInstanceBeenCreated()
{
  return PluginLoader::has_unmananged_instance_been_created_;
}

PluginLoader::PluginLoader(const std::string& library_path, bool ondemand_load_unload)
    : ondemand_load_unload_(ondemand_load_unload), library_path_(library_path), load_ref_count_(0), plugin_ref_count_(0)
{
  namespace fs = std::filesystem;

  fs::path p = fs::path(library_path_);
  fs::path current = fs::path(".");
  fs::path current_abs = canonical(current);
  fs::path library_abs = current_abs / library_path_;
  const fs::path const_library_path = (p.is_relative() ? library_abs : p).lexically_normal();
  const std::string const_library_path_string = const_library_path.string();
  const fs::path route_path = const_library_path.parent_path();
  const std::string file_name_string = const_library_path.filename().string();

  fs::path temp_path;
  if (!exists(temp_path = const_library_path) &&
      !exists(temp_path = const_library_path_string + SharedLibrary::suffix()) &&
      !exists(temp_path = const_library_path_string + SharedLibrary::suffix_d()) &&
      !exists(temp_path = route_path / (SharedLibrary::prefix() + file_name_string + SharedLibrary::suffix())) &&
      !exists(temp_path = route_path / (SharedLibrary::prefix() + file_name_string + SharedLibrary::suffix_d())) &&
      !exists(temp_path = route_path / (SharedLibrary::prefix() + file_name_string)))
  {
    logDebug("library_path maybe not exists!");
  }
  else
  {
    library_path_ = temp_path.string();
  }

//  stdboost::trim(library_path_);
//  const std::string const_library_path_ = library_path_;
//  const std::string route_path = const_library_path_.substr(0, const_library_path_.find_last_of('/') + 1);
//  const std::string file_name = const_library_path_.substr(const_library_path_.find_last_of('/') + 1);
//
//  if (!is_file_exists(library_path_ = const_library_path_) &&
//      !is_file_exists(library_path_ = const_library_path_ + SharedLibrary::suffix()) &&
//      !is_file_exists(library_path_ = const_library_path_ + SharedLibrary::suffix_d()) &&
//      !is_file_exists(library_path_ = route_path + SharedLibrary::prefix() + file_name + SharedLibrary::suffix()) &&
//      !is_file_exists(library_path_ = route_path + SharedLibrary::prefix() + file_name + SharedLibrary::suffix_d()) &&
//      !is_file_exists(library_path_ = route_path + SharedLibrary::prefix() + file_name))
//  {
//    logDebug("library_path maybe not exists!");
//    library_path_ = library_path;
//  }

  logDebug("plugin_loader.PluginLoader: "
           "Constructing new PluginLoader (%p) bound to library %s.",
           this, library_path.c_str());
  if (!isOnDemandLoadUnloadEnabled())
  {
    loadLibrary();
  }
}

PluginLoader::~PluginLoader()
{
  logDebug("%s", "plugin_loader.PluginLoader: "
                 "Destroying class loader, unloading associated library...\n");
  unloadLibrary();  // TODO(mikaelarguedas): while(unloadLibrary() > 0){} ??
}

bool PluginLoader::isLibraryLoaded()
{
  return plugin_loader::impl::isLibraryLoaded(getLibraryPath(), this);
}

bool PluginLoader::isLibraryLoadedByAnyClassloader()
{
  return plugin_loader::impl::isLibraryLoadedByAnybody(getLibraryPath());
}

void PluginLoader::loadLibrary()
{
  std::unique_lock<std::recursive_mutex> lock(load_ref_count_mutex_);
  load_ref_count_ = load_ref_count_ + 1;
  plugin_loader::impl::loadLibrary(getLibraryPath(), this);
}

int PluginLoader::unloadLibrary()
{
  return unloadLibraryInternal(true);
}

int PluginLoader::unloadLibraryInternal(bool lock_plugin_ref_count)
{
  std::unique_lock<std::recursive_mutex> load_ref_lock(load_ref_count_mutex_);
  std::unique_lock<std::recursive_mutex> plugin_ref_lock;
  if (lock_plugin_ref_count)
  {
    plugin_ref_lock = std::unique_lock<std::recursive_mutex>(plugin_ref_count_mutex_);
  }

  if (plugin_ref_count_ > 0)
  {
    logWarn("%s", "plugin_loader.PluginLoader: "
                  "SEVERE WARNING!!! Attempting to unload library while objects created by this loader "
                  "exist in the heap! "
                  "You should delete your objects before attempting to unload the library or "
                  "destroying the PluginLoader. The library will NOT be unloaded.");
  }
  else
  {
    load_ref_count_ = load_ref_count_ - 1;
    if (0 == load_ref_count_)
    {
      plugin_loader::impl::unloadLibrary(getLibraryPath(), this);
    }
    else if (load_ref_count_ < 0)
    {
      load_ref_count_ = 0;
    }
  }
  return load_ref_count_;
}

}  // namespace plugin_loader

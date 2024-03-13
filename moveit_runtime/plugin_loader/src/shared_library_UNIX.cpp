#include <string>
#include <mutex>
#include <dlfcn.h>
#include "plugin_loader/shared_library.hpp"

namespace plugin_loader
{
void SharedLibrary::load(const std::string& path, int flags)
{
  std::unique_lock<std::mutex> lock(_mutex);

  if (_handle)
  {
    throw plugin_loader::LibraryLoadException("Library already loaded: " + path);
  }
  int realFlags = RTLD_LAZY;
  if (flags & SHLIB_LOCAL)
    realFlags |= RTLD_LOCAL;
  else
    realFlags |= RTLD_GLOBAL;
  _handle = dlopen(path.c_str(), realFlags);
  if (!_handle)
  {
    const char* err = dlerror();
    std::cerr << err << std::endl;
    throw plugin_loader::LibraryLoadException("Could not load library: " + path + " " + std::string(err));
  }
  _path = path;
}

void SharedLibrary::unload()
{
  std::unique_lock<std::mutex> lock(_mutex);

  if (_handle)
  {
    dlclose(_handle);
    _handle = 0;
  }
}

void* SharedLibrary::findSymbol(const std::string& name)
{
  std::unique_lock<std::mutex> lock(_mutex);

  void* result = 0;
  if (_handle)
  {
    result = dlsym(_handle, name.c_str());
  }
  return result;
}

}  // namespace plugin_loader

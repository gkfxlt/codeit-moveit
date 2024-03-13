#include <string>
#include <mutex>
#include "plugin_loader/shared_library.hpp"

#include <Windows.h>

namespace plugin_loader {
  void SharedLibrary::load(const std::string &path, int flags) {
    std::unique_lock<std::mutex> lock(_mutex);

    if (_handle) {
      throw plugin_loader::LibraryLoadException("Library already loaded: " + path);
    }

    _handle = LoadLibrary(path.c_str());

    if (!_handle) {
      const char *err = nullptr;
      throw plugin_loader::LibraryLoadException("Could not load library: " + (err ? std::string(err) : path));
    }
    _path = path;
  }

  void SharedLibrary::unload() {
    std::unique_lock<std::mutex> lock(_mutex);

    if (_handle) {
      FreeLibrary((HMODULE) _handle);
      _handle = 0;
    }
  }

  void *SharedLibrary::findSymbol(const std::string &name) {
    std::unique_lock<std::mutex> lock(_mutex);

    void *result = 0;
    if (_handle) {
      result = GetProcAddress((HMODULE) _handle, name.c_str());
    }
    return result;
  }

}  // namespace plugin_loader

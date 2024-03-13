#include "plugins.h"

#include "plugin_loader/plugin_loader.hpp"

PLUGIN_LOADER_REGISTER_CLASS(animal::Dog, Base)
PLUGIN_LOADER_REGISTER_CLASS(animal::Cat, Base)
PLUGIN_LOADER_REGISTER_CLASS(animal::Duck, Base)
PLUGIN_LOADER_REGISTER_CLASS(animal::Cow, Base)
PLUGIN_LOADER_REGISTER_CLASS(animal::Sheep, Base)

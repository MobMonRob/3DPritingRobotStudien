#include "Atlas.hpp"
#include <spdlog/spdlog.h>

int main()
{
    Atlas::initLogger(spdlog::level::debug);

    return 0;
}

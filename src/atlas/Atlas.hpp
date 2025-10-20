#pragma once
#include <filesystem>
#include <spdlog/common.h>

static const std::filesystem::path kLoggingDirectory = "../../data/logs";
static const std::string kLoggingFilePrefix = "slicer_";
static const std::string kLoggingFileExtension = ".log";
static const std::string kLoggerName = "SlicerLogger";

namespace Atlas
{
    void initLogger(
        spdlog::level::level_enum aLogLevel = spdlog::level::info);
}

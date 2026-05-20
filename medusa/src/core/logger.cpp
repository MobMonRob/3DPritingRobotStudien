/**
 * @file logger.cpp
 * @brief Implementation of the global asynchronous spdlog logger.
 */

#include "logger.h"

#include <spdlog/async.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <filesystem>

std::shared_ptr<spdlog::logger> Logger::sLogger;

bool Logger::init(const char* logsDirectory, const spdlog::level::level_enum consoleLevel,
                  const spdlog::level::level_enum fileLevel)
{
    try
    {
        std::error_code fs_error;
        if (logsDirectory && *logsDirectory)
            std::filesystem::create_directories(logsDirectory, fs_error);

        constexpr std::size_t kQueueSize = 8192;
        constexpr std::size_t kThreadCount = 1;
        spdlog::init_thread_pool(kQueueSize, kThreadCount);

        const auto kConsoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        kConsoleSink->set_level(consoleLevel);
        kConsoleSink->set_pattern("%^[%Y-%m-%d %T.%e] [%l] %v%$");

        const std::filesystem::path kLogFilePath = std::filesystem::path(logsDirectory) / "medusa.log";
        const auto kFileSink =
            std::make_shared<spdlog::sinks::rotating_file_sink_mt>(kLogFilePath.string(), 5 * 1024 * 1024, 3);
        kFileSink->set_level(fileLevel);
        kFileSink->set_pattern("[%Y-%m-%d %T.%e] [%l] [thread %t] %v");

        std::vector<spdlog::sink_ptr> sinks{kConsoleSink, kFileSink};

        sLogger = std::make_shared<spdlog::async_logger>("app", sinks.begin(), sinks.end(), spdlog::thread_pool(),
                                                         spdlog::async_overflow_policy::block);

        sLogger->set_level(spdlog::level::trace);
        sLogger->flush_on(spdlog::level::warn);

        spdlog::register_logger(sLogger);
        spdlog::set_default_logger(sLogger);

        sLogger->info("Async logger initialized (console={}, file={})", spdlog::level::to_string_view(consoleLevel),
                      spdlog::level::to_string_view(fileLevel));

        if (fs_error)
            sLogger->warn("Could not create logs directory '{}': {}", logsDirectory, fs_error.message());

        return true;
    }
    catch (const std::exception& ex)
    {
        spdlog::error("Failed to initialize logger: {}", ex.what());
        return false;
    }
}

void Logger::shutdown()
{
    if (sLogger)
    {
        sLogger->flush();
        spdlog::drop(sLogger->name());
        sLogger.reset();
    }
    spdlog::shutdown();
}

std::shared_ptr<spdlog::logger> Logger::get()
{
    return sLogger;
}
#include "Atlas.hpp"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <chrono>
#include <iomanip>
#include <sstream>

namespace Atlas
{

    static std::string timestampNow()
    {
        const std::time_t kNowC = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::tm tm{};

#ifdef _WIN32
        localtime_s(&tm, &now_c);
#else
        localtime_r(&kNowC, &tm);
#endif

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
        return oss.str();
    }

    void initLogger(const spdlog::level::level_enum aLogLevel)
    {
        try
        {
            if (!std::filesystem::exists(kLoggingDirectory))
            {
                std::filesystem::create_directories(kLoggingDirectory);
            }

            const auto kFileName = kLoggingFilePrefix + timestampNow() + kLoggingFileExtension;
            const auto kLogFilePath = kLoggingDirectory / kFileName;

            const auto kConsoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

            const auto kFileSink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                (kLoggingDirectory / kFileName).string(), true);

            std::vector<spdlog::sink_ptr> sinks{kConsoleSink, kFileSink};
            const auto kLogger = std::make_shared<spdlog::logger>(kLoggerName, sinks.begin(), sinks.end());
            kLogger->set_level(aLogLevel);

            spdlog::set_default_logger(kLogger);
            spdlog::info("Logger initialized → {}", kFileName);
            spdlog::info("Log level set to {}", spdlog::level::to_string_view(aLogLevel));
        }
        catch (const std::exception& ex)
        {
            fprintf(stderr, "Logger init failed: %s\n", ex.what());
        }
    }

}

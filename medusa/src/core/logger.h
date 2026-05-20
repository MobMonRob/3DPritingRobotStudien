#pragma once

#include <memory>

#include <spdlog/logger.h>

/**
 * @file logger.h
 * @brief Central logging facility (thin wrapper around spdlog).
 *
 * The application uses a single global logger instance named "app". Initialize it once at
 * startup via Logger::init and shut it down via Logger::shutdown.
 *
 * Convenience logging macros are provided (e.g. MEDUSA_INFO). They are safe to call even
 * if the logger has not been initialized yet (in that case they do nothing).
 *
 * @note spdlog supports compile-time log filtering via SPDLOG_ACTIVE_LEVEL.
 *       When enabled, calls below the configured level may be completely removed from the
 *       compiled binary (not just filtered at runtime). This project sets SPDLOG_ACTIVE_LEVEL
 *       via CMake depending on the build type.
 */

#ifndef MEDUSA_SRC_CORE_LOGGER_H_
#define MEDUSA_SRC_CORE_LOGGER_H_

#include <memory>
#include <spdlog/logger.h>

/**
 * @brief Static logging utility.
 *
 * This class owns the globally configured spdlog logger instance.
 *
 * Typical usage:
 * @code{.cpp}
 * int main() {
 *     Logger::Init();
 *     MEDUSA_INFO("Hello");
 *     Logger::Shutdown();
 * }
 * @endcode
 */
class Logger
{
public:
    /**
   * @brief Initializes the global application logger.
   *
   * The logger is configured with:
   * - colored console output
   * - rotating file output (under the given directory)
   * - asynchronous processing (background thread) to keep the render loop responsive
   *
   * @param logsDirectory Directory where log files should be placed (created if needed).
   * @param console_level Minimum log level for console output.
   * @param fileLevel Minimum log level for file output.
   * @return True on success.
   */
    static bool init(const char* logsDirectory = "logs",
                     spdlog::level::level_enum console_level = spdlog::level::info,
                     spdlog::level::level_enum fileLevel = spdlog::level::debug);

    /**
   * @brief Flushes and shuts down logging.
   *
   * After calling this, Logger::get returns a null pointer and the convenience
   * macros become no-ops.
   */
    static void shutdown();

    /**
   * @brief Returns the global logger instance.
   *
   * Prefer the convenience macros for day-to-day usage.
   *
   * @return Shared logger pointer, or null if Logger::Init was not called.
   */
    static std::shared_ptr<spdlog::logger> get();

private:
    /** @brief Global logger instance (null until initialized). */
    static std::shared_ptr<spdlog::logger> sLogger;
};

/**
 * @name Convenience logging macros
 * @brief Shorthand wrappers around the global logger.
 *
 * These macros check whether the logger exists before logging. This avoids crashes
 * when logging is called very early during startup.
 *
 * @{
 */
#define MEDUSA_TRACE(...)                                        \
  do {                                                          \
    if (auto _lg = ::Logger::get()) _lg->trace(__VA_ARGS__);    \
  } while (0)
#define MEDUSA_DEBUG(...)                                        \
  do {                                                          \
    if (auto _lg = ::Logger::get()) _lg->debug(__VA_ARGS__);    \
  } while (0)
#define MEDUSA_INFO(...)                                         \
  do {                                                          \
    if (auto _lg = ::Logger::get()) _lg->info(__VA_ARGS__);     \
  } while (0)
#define MEDUSA_WARN(...)                                         \
  do {                                                          \
    if (auto _lg = ::Logger::get()) _lg->warn(__VA_ARGS__);     \
  } while (0)
#define MEDUSA_ERROR(...)                                        \
  do {                                                          \
    if (auto _lg = ::Logger::get()) _lg->error(__VA_ARGS__);    \
  } while (0)
#define MEDUSA_CRITICAL(...)                                     \
  do {                                                          \
    if (auto _lg = ::Logger::get()) _lg->critical(__VA_ARGS__); \
  } while (0)
/** @} */

#endif  // MEDUSA_SRC_CORE_LOGGER_H_
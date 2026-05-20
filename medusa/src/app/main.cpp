#include "app.h"
#include "logger.h"

/**
 * @file main.cpp
 * @brief Application entry point.
 */

/**
 * @brief Starts the STL viewer application.
 *
 * Initializes the global logging facility before constructing the application.
 *
 * Forwards command line arguments to @ref App::run.
 *
 * @param argc Number of command line arguments.
 * @param argv Argument vector.
 * @return Process exit code (0 on success).
 */
int main(const int argc, char** argv)
{
    Logger::init();

    App app;
    const int kExitCode = app.run(argc, argv);

    Logger::shutdown();
    return kExitCode;
}
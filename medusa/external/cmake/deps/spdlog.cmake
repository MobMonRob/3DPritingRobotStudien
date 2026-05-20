include_guard(GLOBAL)

## -------------------------------------
## spdlog (logging)
## -------------------------------------
# Repo: https://github.com/gabime/spdlog

add_library(medusa_sys_spdlog INTERFACE)

find_package(spdlog QUIET CONFIG)

if (spdlog_FOUND)
    medusa_log("spdlog: found system installation (version ${spdlog_VERSION})")

    if (TARGET spdlog::spdlog)
        target_link_libraries(medusa_sys_spdlog INTERFACE spdlog::spdlog)
    elseif (TARGET spdlog)
        target_link_libraries(medusa_sys_spdlog INTERFACE spdlog)
    else ()
        medusa_log("spdlog found but targets are undefined" WARNING)
    endif ()

else ()
    medusa_log("spdlog: not found on system -> Fetching from Git...")
    medusa_log("spdlog: configure (tag=v1.17.0, std_format=${EXT_SPDLOG_USE_STD_FORMAT})")

    FetchContent_Declare(
            spdlog
            GIT_REPOSITORY https://github.com/gabime/spdlog.git
            GIT_TAG v1.17.0
            GIT_SHALLOW ${_FETCH_SHALLOW}
            GIT_PROGRESS TRUE
            GIT_SUBMODULES ""
    )

    set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "" FORCE)
    set(SPDLOG_USE_STD_FORMAT ${EXT_SPDLOG_USE_STD_FORMAT} CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(SPDLOG_BUILD_BENCH OFF CACHE BOOL "" FORCE)
    set(SPDLOG_INSTALL OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(spdlog)

    if (TARGET spdlog::spdlog)
        medusa_log("spdlog: fetch target ready" VERBOSE)
        target_link_libraries(medusa_sys_spdlog INTERFACE spdlog::spdlog)
    else ()
        medusa_log("spdlog: expected target 'spdlog::spdlog' missing after fetch" FATAL_ERROR)
    endif ()
endif ()

if (EXT_SPDLOG_NO_EXCEPTIONS)
    medusa_log("spdlog: SPDLOG_NO_EXCEPTIONS enabled" NOTICE)
    target_compile_definitions(medusa_sys_spdlog INTERFACE SPDLOG_NO_EXCEPTIONS)
endif ()

add_library(ext::spdlog ALIAS medusa_sys_spdlog)
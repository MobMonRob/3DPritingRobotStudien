include_guard(GLOBAL)

## -------------------------------------
## nlohmann/json (header-only JSON)
## -------------------------------------
# Repo: https://github.com/nlohmann/json

# Try to find system-installed nlohmann_json first
find_package(nlohmann_json QUIET CONFIG)

if (nlohmann_json_FOUND)
    medusa_log("nlohmann_json: found system installation (version ${nlohmann_json_VERSION})")
    if (TARGET nlohmann_json::nlohmann_json)
        add_library(ext::json ALIAS nlohmann_json::nlohmann_json)
    elseif (TARGET nlohmann_json)
        add_library(ext::json ALIAS nlohmann_json)
    else ()
        medusa_log("nlohmann_json was found but no suitable target exists" , WARNING)
    endif ()
else ()
    # Fetch nlohmann_json if not found on system
    medusa_log("nlohmann_json: configure (tag=v3.12.0, shallow=${_FETCH_SHALLOW})")
    FetchContent_Declare(
            nlohmann_json
            GIT_REPOSITORY https://github.com/nlohmann/json.git
            GIT_TAG v3.12.0
            GIT_SHALLOW ${_FETCH_SHALLOW}
            GIT_PROGRESS TRUE
            GIT_SUBMODULES ""
    )

    FetchContent_MakeAvailable(nlohmann_json)
    if (TARGET nlohmann_json)
        medusa_log("nlohmann_json: target 'nlohmann_json' ready" , VERBOSE)
        add_library(ext::json ALIAS nlohmann_json)
    else ()
        medusa_log("nlohmann_json was fetched, but target 'nlohmann_json' is missing" , WARNING)
    endif ()
endif ()

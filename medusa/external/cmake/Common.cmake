# Central configuration for external dependencies.
# Included by external/CMakeLists.txt.

include_guard(GLOBAL)

include(FetchContent)

# -------------------------------------
# FetchContent defaults
# -------------------------------------
option(FETCH_USE_SHALLOW "Use shallow git clones for FetchContent dependencies" ON)
option(FETCH_ALLOW_UPDATES "Allow network updates after the initial fetch" OFF)

# Some git hosts / filesystems occasionally fail when multiple FetchContent
# populate steps clone in parallel (tmp_pack_* errors). Keep it serial by default.
option(MEDUSA_FETCH_PARALLEL "Allow parallel FetchContent populates" OFF)
if (NOT MEDUSA_FETCH_PARALLEL)
    set(FETCHCONTENT_QUIET ON)
    set(FETCHCONTENT_PARALLEL OFF)
else ()
    set(FETCHCONTENT_QUIET ON)
endif ()

mark_as_advanced(FETCH_USE_SHALLOW FETCH_ALLOW_UPDATES MEDUSA_FETCH_PARALLEL)

medusa_log("External deps config: shallow=${FETCH_USE_SHALLOW}, allow_updates=${FETCH_ALLOW_UPDATES}, parallel=${MEDUSA_FETCH_PARALLEL}")
if (FETCH_ALLOW_UPDATES)
    medusa_log("FETCH_ALLOW_UPDATES=ON -> less reproducible builds possible (network updates after initial fetch)." , WARNING)
endif ()
if (NOT MEDUSA_FETCH_PARALLEL)
    medusa_log("FetchContent parallel disabled (more stable on some filesystems/hosts)." , NOTICE)
else ()
    medusa_log("FetchContent parallel enabled." , VERBOSE)
endif ()

# Deterministic timestamps for downloaded archives
if (POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif ()

# Prefer reproducible, faster builds by disconnecting updates by default
if (NOT FETCH_ALLOW_UPDATES)
    set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
endif ()

# Helper used for GIT_SHALLOW
set(_FETCH_SHALLOW ${FETCH_USE_SHALLOW})

# -------------------------------------
# Project-wide dependency switches (namespaced)
# -------------------------------------
option(EXT_IMGUI_WITH_DEMO "Include ImGui demo window and code" ON)
option(EXT_SPDLOG_USE_STD_FORMAT "Use std::format backend in spdlog (requires C++20)" OFF)
option(EXT_SPDLOG_NO_EXCEPTIONS "Build spdlog without exceptions (smaller/faster, but no try/catch)" OFF)

medusa_log("Dependency options: EXT_IMGUI_WITH_DEMO=${EXT_IMGUI_WITH_DEMO}, EXT_SPDLOG_USE_STD_FORMAT=${EXT_SPDLOG_USE_STD_FORMAT}, EXT_SPDLOG_NO_EXCEPTIONS=${EXT_SPDLOG_NO_EXCEPTIONS}")

# If the project compiles with C++17 (or older), force-disable std::format in spdlog
if (DEFINED CMAKE_CXX_STANDARD AND CMAKE_CXX_STANDARD LESS 20)
    if (EXT_SPDLOG_USE_STD_FORMAT)
        medusa_log("EXT_SPDLOG_USE_STD_FORMAT requested, but CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD} < 20 -> disabling." , NOTICE)
    else ()
        medusa_log("CMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD} < 20 -> keeping std::format backend in spdlog disabled." , VERBOSE)
    endif ()
    set(EXT_SPDLOG_USE_STD_FORMAT OFF CACHE BOOL "Disable std::format on < C++20" FORCE)
endif ()

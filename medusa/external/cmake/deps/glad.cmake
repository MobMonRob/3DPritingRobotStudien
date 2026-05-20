include_guard(GLOBAL)

## -------------------------------------
## GLAD (Hybrid: System or Vendored)
## -------------------------------------
# Repo: https://github.com/Dav1dde/glad

find_library(GLAD_SYS_LIB NAMES glad PATHS /usr/local/lib NO_DEFAULT_PATH)
find_path(GLAD_SYS_INCLUDE NAMES glad/glad.h PATHS /usr/local/include NO_DEFAULT_PATH)

if (GLAD_SYS_LIB AND GLAD_SYS_INCLUDE)
    medusa_log("GLAD: found pre-compiled Docker library")
    add_library(glad_lib STATIC IMPORTED GLOBAL)

    set_target_properties(glad_lib PROPERTIES
            IMPORTED_LOCATION "${GLAD_SYS_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${GLAD_SYS_INCLUDE}"
    )

    if (UNIX)
        target_link_libraries(glad_lib INTERFACE dl)
    endif ()

else ()
    medusa_log("GLAD: using vendored sources (external/glad)")

    set(GLAD_ROOT "${PROJECT_SOURCE_DIR}/external/glad")

    if (NOT EXISTS "${GLAD_ROOT}/src/glad.c")
        medusa_log("GLAD source missing! Expected at: ${GLAD_ROOT}/src/glad.c" FATAL_ERROR)
    endif ()

    add_library(glad_lib STATIC "${GLAD_ROOT}/src/glad.c")

    target_include_directories(glad_lib SYSTEM PUBLIC "${GLAD_ROOT}/include")

    set_target_properties(glad_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

    if (UNIX)
        target_link_libraries(glad_lib PUBLIC dl)
    endif ()
endif ()

if (NOT TARGET ext::glad)
    add_library(ext::glad ALIAS glad_lib)
endif ()

medusa_log("GLAD: target 'ext::glad' ready" VERBOSE)
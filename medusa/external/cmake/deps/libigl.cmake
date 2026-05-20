include_guard(GLOBAL)

## -------------------------------------
## libigl (geometry processing, header-only)
## -------------------------------------
# Repo: https://github.com/libigl/libigl

medusa_log("libigl: configure (tag=v2.5.0, fetched)")

FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG v2.5.0
        GIT_SHALLOW ${FETCH_USE_SHALLOW}
        GIT_PROGRESS TRUE
)

# Disable libigl extras we don't need.
set(LIBIGL_BUILD_TESTS       OFF CACHE BOOL "" FORCE)
set(LIBIGL_BUILD_TUTORIALS   OFF CACHE BOOL "" FORCE)
set(LIBIGL_INSTALL           OFF CACHE BOOL "" FORCE)
set(LIBIGL_USE_STATIC_LIBRARY OFF CACHE BOOL "" FORCE)

# Tell libigl not to fetch its own Eigen — we already provide Eigen3::Eigen.
set(LIBIGL_WITH_OPENGL       OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_OPENGL_GLFW  OFF CACHE BOOL "" FORCE)
set(LIBIGL_WITH_IMGUI        OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(libigl)

# Wrap in a medusa-namespaced interface target.
add_library(medusa_sys_libigl INTERFACE)
if (TARGET igl::core)
    medusa_log("libigl: target igl::core ready" VERBOSE)
    target_link_libraries(medusa_sys_libigl INTERFACE igl::core)
else ()
    medusa_log("igl::core target missing — falling back to include path (${libigl_SOURCE_DIR}/include)" WARNING)
    target_include_directories(medusa_sys_libigl INTERFACE ${libigl_SOURCE_DIR}/include)
endif ()

add_library(ext::libigl ALIAS medusa_sys_libigl)

include_guard(GLOBAL)

## -------------------------------------
## Eigen (linear algebra, header-only)
## -------------------------------------
# Repo: https://gitlab.com/libeigen/eigen

medusa_log("Eigen: configure (tag=3.4.0, fetched)")

FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 5.0.1
        GIT_SHALLOW ${FETCH_USE_SHALLOW}
        GIT_PROGRESS TRUE
)

# Eigen's own CMake builds tests / blas / lapack / docs by default — disable.
set(BUILD_TESTING                  OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC                OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING            OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG          OFF CACHE BOOL "" FORCE)
set(EIGEN_LEAVE_TEST_IN_ALL_TARGET OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(eigen)

# Make the target available either through the canonical name or fallback path.
add_library(medusa_sys_eigen INTERFACE)
if (TARGET Eigen3::Eigen)
    medusa_log("Eigen: target Eigen3::Eigen ready" VERBOSE)
    target_link_libraries(medusa_sys_eigen INTERFACE Eigen3::Eigen)
else ()
    medusa_log("Eigen3::Eigen target missing — falling back to include path (${eigen_SOURCE_DIR})" WARNING)
    target_include_directories(medusa_sys_eigen INTERFACE ${eigen_SOURCE_DIR})
    # Provide the canonical alias so libigl's recipe sees it and skips its own fetch.
    add_library(Eigen3_Eigen INTERFACE)
    target_include_directories(Eigen3_Eigen INTERFACE ${eigen_SOURCE_DIR})
    add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)
endif ()

add_library(ext::eigen ALIAS medusa_sys_eigen)

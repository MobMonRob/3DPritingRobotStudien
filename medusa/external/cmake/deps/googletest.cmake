include_guard(GLOBAL)

## -------------------------------------
## GoogleTest (unit testing)
## -------------------------------------
# Repo: https://github.com/google/googletest

add_library(medusa_sys_gtest INTERFACE)
add_library(medusa_sys_gtest_main INTERFACE)

find_package(GTest QUIET CONFIG)

if (GTest_FOUND)
    medusa_log("googletest: found system installation (version ${GTest_VERSION})")

    if (TARGET GTest::gtest)
        target_link_libraries(medusa_sys_gtest INTERFACE GTest::gtest)
    elseif (TARGET gtest)
        target_link_libraries(medusa_sys_gtest INTERFACE gtest)
    else ()
        medusa_log("googletest: found GTest package but target 'gtest' is missing" WARNING)
    endif ()

    if (TARGET GTest::gtest_main)
        target_link_libraries(medusa_sys_gtest_main INTERFACE GTest::gtest_main)
    elseif (TARGET gtest_main)
        target_link_libraries(medusa_sys_gtest_main INTERFACE gtest_main)
    endif ()

else ()
    medusa_log("googletest: configure (tag=v1.17.0, shallow=${_FETCH_SHALLOW})")

    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG v1.17.0
            GIT_SHALLOW ${_FETCH_SHALLOW}
            GIT_PROGRESS TRUE
            GIT_SUBMODULES ""
    )

    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    set(BUILD_GTEST ON CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(googletest)

    if (TARGET gtest)
        medusa_log("googletest: fetch target 'gtest' ready" VERBOSE)
        target_link_libraries(medusa_sys_gtest INTERFACE gtest)

        if (MSVC)
            target_compile_options(gtest PRIVATE /w)
        else ()
            target_compile_options(gtest PRIVATE -w)
        endif ()
    else ()
        medusa_log("googletest: fetch failed, target 'gtest' missing" FATAL_ERROR)
    endif ()

    if (TARGET gtest_main)
        medusa_log("googletest: fetch target 'gtest_main' ready" VERBOSE)
        target_link_libraries(medusa_sys_gtest_main INTERFACE gtest_main)

        # Auch hier Warnungen unterdrücken
        if (MSVC)
            target_compile_options(gtest_main PRIVATE /w)
        else ()
            target_compile_options(gtest_main PRIVATE -w)
        endif ()
    endif ()

endif ()

add_library(ext::gtest ALIAS medusa_sys_gtest)
add_library(ext::gtest_main ALIAS medusa_sys_gtest_main)
include_guard(GLOBAL)

## -------------------------------------
## GLFW (window + input)
## -------------------------------------
# Repo: https://github.com/glfw/glfw

add_library(medusa_sys_glfw INTERFACE)

# Try to find system-installed GLFW first
find_package(glfw3 QUIET CONFIG)

if (glfw3_FOUND)
    medusa_log("GLFW: found system installation (version ${glfw3_VERSION})")

    if (TARGET glfw)
        target_link_libraries(medusa_sys_glfw INTERFACE glfw)
    elseif (TARGET glfw3::glfw)
        target_link_libraries(medusa_sys_glfw INTERFACE glfw3::glfw)
    else ()
        medusa_log("GLFW found but target is missing (checked 'glfw' and 'glfw3::glfw')" WARNING)
    endif ()

else ()
    # Fetch GLFW if not found on system
    medusa_log("GLFW: configure (tag=3.4, shallow=${_FETCH_SHALLOW})")

    FetchContent_Declare(
            glfw
            GIT_REPOSITORY https://github.com/glfw/glfw.git
            GIT_TAG 3.4
            GIT_SHALLOW ${_FETCH_SHALLOW}
            GIT_PROGRESS TRUE
            GIT_SUBMODULES ""
    )

    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(glfw)

    if (TARGET glfw)
        medusa_log("GLFW: target 'glfw' ready" VERBOSE)

        target_link_libraries(medusa_sys_glfw INTERFACE glfw)

        medusa_log("GLFW: disabling warnings for target 'glfw'" VERBOSE)
        if (MSVC)
            target_compile_options(glfw PRIVATE /w)
        else ()
            target_compile_options(glfw PRIVATE -w)
        endif ()

    else ()
        medusa_log("GLFW was fetched, but target 'glfw' is missing" WARNING)
    endif ()
endif ()

add_library(ext::glfw ALIAS medusa_sys_glfw)
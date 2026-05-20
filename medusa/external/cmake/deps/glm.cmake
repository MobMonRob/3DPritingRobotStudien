include_guard(GLOBAL)

## -------------------------------------
## GLM (math library)
## -------------------------------------
# Repo: https://github.com/g-truc/glm

add_library(medusa_sys_glm INTERFACE)

# Try to find system-installed GLM first
find_package(glm QUIET CONFIG)

if (glm_FOUND)
    medusa_log("GLM: found system installation (version ${glm_VERSION})")

    if (TARGET glm::glm)
        target_link_libraries(medusa_sys_glm INTERFACE glm::glm)
    elseif (TARGET glm)
        target_link_libraries(medusa_sys_glm INTERFACE glm)
    else ()
        medusa_log("GLM was found but no suitable target exists" WARNING)
    endif ()

else ()
    # Fetch GLM if not found on system
    medusa_log("GLM: configure (tag=1.0.3, shallow=${_FETCH_SHALLOW})")

    FetchContent_Declare(
            glm
            GIT_REPOSITORY https://github.com/g-truc/glm.git
            GIT_TAG 1.0.3
            GIT_SHALLOW ${_FETCH_SHALLOW}
            GIT_PROGRESS TRUE
            GIT_SUBMODULES ""
    )
    set(GLM_TEST_ENABLE OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(glm)

    if (TARGET glm)
        medusa_log("GLM: target 'glm' ready" VERBOSE)

        target_link_libraries(medusa_sys_glm INTERFACE glm)

        get_target_property(GLM_INCLUDE_DIRS glm INTERFACE_INCLUDE_DIRECTORIES)
        if(GLM_INCLUDE_DIRS)
            target_include_directories(medusa_sys_glm SYSTEM INTERFACE ${GLM_INCLUDE_DIRS})
        endif()

    else ()
        medusa_log("GLM was fetched, but target 'glm' is missing" WARNING)
    endif ()
endif ()

add_library(ext::glm ALIAS medusa_sys_glm)
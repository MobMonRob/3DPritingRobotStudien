include_guard(GLOBAL)

## -------------------------------------
## Assimp (3D asset import)
## -------------------------------------
# Repo: https://github.com/assimp/assimp

add_library(medusa_sys_assimp INTERFACE)

find_package(assimp QUIET CONFIG)

if (assimp_FOUND)
    medusa_log("Assimp: found system installation (version ${assimp_VERSION})")

    if (TARGET assimp::assimp)
        target_link_libraries(medusa_sys_assimp INTERFACE assimp::assimp)
    elseif (TARGET assimp)
        target_link_libraries(medusa_sys_assimp INTERFACE assimp)
    else ()
        medusa_log("Assimp found but targets are undefined" WARNING)
    endif ()

else ()
    medusa_log("Assimp: configure (tag=v6.0.4)")

    FetchContent_Declare(
            assimp
            GIT_REPOSITORY https://github.com/assimp/assimp.git
            GIT_TAG v6.0.4
            GIT_SHALLOW FALSE
            GIT_PROGRESS TRUE
    )

    set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
    set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT ON CACHE BOOL "" FORCE)
    set(ASSIMP_NO_EXPORT ON CACHE BOOL "" FORCE)
    set(ASSIMP_BUILD_ZLIB OFF CACHE BOOL "" FORCE)

    FetchContent_MakeAvailable(assimp)

    if (TARGET assimp)
        medusa_log("Assimp: fetch target ready" VERBOSE)

        target_link_libraries(medusa_sys_assimp INTERFACE assimp)

        medusa_log("Assimp: suppressing warnings" VERBOSE)
        if(MSVC)
            target_compile_options(assimp PRIVATE /w)
        else()
            target_compile_options(assimp PRIVATE -w)
        endif()

    else ()
        medusa_log("Assimp was fetched, but target 'assimp' is missing" FATAL_ERROR)
    endif ()
endif ()

add_library(ext::assimp ALIAS medusa_sys_assimp)
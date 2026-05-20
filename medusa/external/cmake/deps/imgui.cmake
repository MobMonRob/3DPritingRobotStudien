include_guard(GLOBAL)

## -------------------------------------
## Dear ImGui (Docking Branch)
## -------------------------------------
# Repo: https://github.com/ocornut/imgui

find_library(IMGUI_SYS_LIB NAMES imgui_core PATHS /usr/local/lib NO_DEFAULT_PATH)
find_path(IMGUI_SYS_INCLUDE NAMES imgui.h PATHS /usr/local/include/imgui NO_DEFAULT_PATH)

if (IMGUI_SYS_LIB AND IMGUI_SYS_INCLUDE)
    medusa_log("ImGui: found pre-compiled Docker library")

    add_library(imgui_prebuilt STATIC IMPORTED GLOBAL)

    set_target_properties(imgui_prebuilt PROPERTIES
            IMPORTED_LOCATION "${IMGUI_SYS_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${IMGUI_SYS_INCLUDE}"
    )

    add_library(imgui_lib STATIC
            "${IMGUI_SYS_INCLUDE}/backends/imgui_impl_glfw.cpp"
            "${IMGUI_SYS_INCLUDE}/backends/imgui_impl_opengl3.cpp"
    )

    target_link_libraries(imgui_lib PUBLIC imgui_prebuilt)

    target_include_directories(imgui_lib SYSTEM PUBLIC
            "${IMGUI_SYS_INCLUDE}"
            "${IMGUI_SYS_INCLUDE}/backends"
    )

else ()
    medusa_log("ImGui: system lib not found, fetching sources...")

    FetchContent_Declare(
            imgui
            GIT_REPOSITORY https://github.com/ocornut/imgui.git
            GIT_TAG docking
            GIT_SHALLOW FALSE
            GIT_PROGRESS TRUE
    )

    FetchContent_MakeAvailable(imgui)

    set(IMGUI_DIR "${imgui_SOURCE_DIR}")

    set(IMGUI_SOURCES
            ${IMGUI_DIR}/imgui.cpp
            ${IMGUI_DIR}/imgui_draw.cpp
            ${IMGUI_DIR}/imgui_tables.cpp
            ${IMGUI_DIR}/imgui_widgets.cpp
            ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
            ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
    )

    if (EXT_IMGUI_WITH_DEMO)
        list(APPEND IMGUI_SOURCES "${IMGUI_DIR}/imgui_demo.cpp")
    endif ()

    add_library(imgui_lib STATIC ${IMGUI_SOURCES})

    target_include_directories(imgui_lib SYSTEM PUBLIC
            "${IMGUI_DIR}"
            "${IMGUI_DIR}/backends"
    )
endif ()

target_compile_definitions(imgui_lib PUBLIC IMGUI_IMPL_OPENGL_LOADER_GLAD)

if (NOT TARGET ext::glfw OR NOT TARGET ext::glad)
    medusa_log("ImGui: missing dependencies! Ensure 'ext::glfw' and 'ext::glad' are loaded." WARNING)
endif ()

target_link_libraries(imgui_lib PUBLIC ext::glfw ext::glad)

set_target_properties(imgui_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

if (NOT TARGET ext::imgui)
    add_library(ext::imgui ALIAS imgui_lib)
endif ()

medusa_log("ImGui: target 'ext::imgui' ready" VERBOSE)
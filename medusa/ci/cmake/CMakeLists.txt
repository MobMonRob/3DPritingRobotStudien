cmake_minimum_required(VERSION 3.22.1)
project(MedusaDockerBuilder)

find_package(glfw3 REQUIRED)

include(FetchContent)

# -----------------------------------------------------------------------------
# IMGUI (Docking Branch)
# -----------------------------------------------------------------------------
message(STATUS "DockerBuilder: Fetching ImGui (Docking)...")

FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG docking
        GIT_SHALLOW FALSE
        GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(imgui)

set(IMGUI_CORE_SOURCES
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
)

add_library(imgui_core STATIC ${IMGUI_CORE_SOURCES})

target_include_directories(imgui_core PUBLIC
        $<BUILD_INTERFACE:${imgui_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/imgui>
)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

include(GNUInstallDirs)

install(TARGETS imgui_core
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(FILES
        ${imgui_SOURCE_DIR}/imgui.h
        ${imgui_SOURCE_DIR}/imconfig.h
        ${imgui_SOURCE_DIR}/imgui_internal.h
        ${imgui_SOURCE_DIR}/imstb_rectpack.h
        ${imgui_SOURCE_DIR}/imstb_textedit.h
        ${imgui_SOURCE_DIR}/imstb_truetype.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/imgui
)

install(FILES
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.h
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.h
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3_loader.h
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/imgui/backends
)

message(STATUS "DockerBuilder: ImGui setup complete.")
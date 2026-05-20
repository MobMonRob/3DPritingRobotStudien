# External dependencies for Medusa

# The individual dependencies live in external/cmake/deps/*.cmake.
# Keeping this file small makes it easier to maintain and review.

set(MEDUSA_LOG_CATEGORY "DEPS")

include(${CMAKE_CURRENT_LIST_DIR}/cmake/Common.cmake)

medusa_log("Configuring external dependencies (external/ ...)")
medusa_log("Deps include dir: ${CMAKE_CURRENT_LIST_DIR}/cmake/deps" , VERBOSE)

# -------------------------------------
# Core dependencies
# -------------------------------------
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/glfw.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/glm.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/spdlog.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/glad.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/imgui.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/assimp.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/nlohmann_json.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/eigen.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/libigl.cmake)

# -------------------------------------
# Optional: unit testing
# -------------------------------------
if (MEDUSA_BUILD_TESTS)
    medusa_log("Tests enabled -> loading googletest")
    include(${CMAKE_CURRENT_LIST_DIR}/cmake/deps/googletest.cmake)
else ()
    medusa_log("Tests disabled -> skipping googletest" , DEBUG)
endif ()

# -------------------------------------
# Convenience interface for linking
# -------------------------------------
add_library(ext_deps INTERFACE)
# Keep the concrete targets to match existing usage.
target_link_libraries(ext_deps INTERFACE glfw glm spdlog::spdlog glad_lib imgui_lib assimp nlohmann_json)
add_library(ext::deps ALIAS ext_deps)

medusa_log("ext::deps ready (glfw, glm, spdlog, glad, imgui, assimp, nlohmann_json)")

# ============================================================================
# Top-level CMake configuration for the Medusa project
# This file configures the build system, dependencies, static analysis, and tests.
# ============================================================================

# Minimum required CMake version
cmake_minimum_required(VERSION 3.22.1...4.1.2)

# ----------------------------------------------------------------------------
# Project metadata
# ----------------------------------------------------------------------------
project(Medusa
        VERSION 1.0.0
        LANGUAGES CXX C
        HOMEPAGE_URL ""
        DESCRIPTION ""
)

# Logging helpers (must be included right after project() so medusa_log() is available)
include(cmake/Utils.cmake)
set(MEDUSA_LOG_CATEGORY "ROOT")

medusa_log("Configure ${PROJECT_NAME} ${PROJECT_VERSION}")
medusa_log("Generator: ${CMAKE_GENERATOR}" , VERBOSE)
medusa_log("Compiler: CXX=${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}" , VERBOSE)

# ----------------------------------------------------------------------------
# Compiler settings
# ----------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

medusa_log("C++ standard: C++${CMAKE_CXX_STANDARD}")

# ----------------------------------------------------------------------------
# Performance Optimizations (Mold Linker & CCache)
# ----------------------------------------------------------------------------
# 1. Mold Linker: Faster linking (especially useful in Docker/CI)
find_program(MOLD_LINKER mold)
if (MOLD_LINKER)
    medusa_log("Performance: Found mold linker, activating...")
    add_link_options("-fuse-ld=mold")
else ()
    medusa_log("Performance: Mold linker not found (using default linker)" , VERBOSE)
endif ()

# 2. CCache: Faster recompilation by caching previous build artifacts
find_program(CCACHE_PROGRAM ccache)
if (CCACHE_PROGRAM)
    medusa_log("Performance: Found ccache, activating...")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
else ()
    medusa_log("Performance: CCache not found" , VERBOSE)
endif ()

# ----------------------------------------------------------------------------
# Detect if this is the top-level project
# ----------------------------------------------------------------------------
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(MEDUSA_IS_TOPLEVEL ON)
else ()
    set(MEDUSA_IS_TOPLEVEL OFF)
endif ()

medusa_log("Toplevel: ${MEDUSA_IS_TOPLEVEL}")

# ----------------------------------------------------------------------------
# Build options
# ----------------------------------------------------------------------------
option(MEDUSA_BUILD_APP "Build the GUI Application" ${MEDUSA_IS_TOPLEVEL})
option(MEDUSA_BUILD_TESTS "Build Unit Tests" ${MEDUSA_IS_TOPLEVEL})

medusa_log("Build options: APP=${MEDUSA_BUILD_APP}, TESTS=${MEDUSA_BUILD_TESTS}")

# ----------------------------------------------------------------------------
# External dependencies
# ----------------------------------------------------------------------------
add_subdirectory(external)

medusa_log("External dependencies configured")

# ----------------------------------------------------------------------------
# Main project sources
# ----------------------------------------------------------------------------
add_subdirectory(src)

# ==========================================================================
# DOCUMENTATION TARGET
# ==========================================================================
find_package(Doxygen)

if (DOXYGEN_FOUND)
    medusa_log("Doxygen found: ${DOXYGEN_EXECUTABLE}")
    add_custom_target(doxygen
            COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
    )
else ()
    medusa_log("[Docs] Doxygen not found. Documentation target disabled." , NOTICE)
endif ()

# ============================================================================
# TESTS
# Enable CTest and add test targets if requested
# ============================================================================
if (MEDUSA_BUILD_TESTS)
    medusa_log("Tests enabled")
    include(CTest)
    add_subdirectory(tests)
else ()
    medusa_log("Tests disabled" , VERBOSE)
endif ()
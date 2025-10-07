cmake_minimum_required(VERSION 4.0.0)

project(Slicer
        VERSION 0.0.1
        LANGUAGES CXX
        HOMEPAGE_URL "https://github.com/MobMonRob/3DPritingRobotStudien"
        DESCRIPTION "6D-Slicer"
)

include(CTest)

add_subdirectory(src)
add_subdirectory(tests)
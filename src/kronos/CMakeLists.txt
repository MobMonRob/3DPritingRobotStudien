add_library(Kronos STATIC
        Kronos.cpp
        Kronos.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(medusa STATIC
        medusa.cpp
        Medusa.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

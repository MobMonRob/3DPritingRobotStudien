add_library(Medusa STATIC
        Medusa.cpp
        Medusa.hpp
)

target_link_libraries(Medusa PUBLIC
        Kronos
        spdlog::spdlog
)

target_include_directories(Medusa PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(Kronos STATIC
        Kronos.cpp
        Kronos.hpp
)

target_link_libraries(Kronos PUBLIC
        Medusa
        spdlog::spdlog
)

target_include_directories(Kronos PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

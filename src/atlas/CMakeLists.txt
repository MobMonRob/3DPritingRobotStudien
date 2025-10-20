add_library(Atlas STATIC
        Atlas.cpp
        Atlas.hpp
)

target_link_libraries(Atlas PUBLIC
        Medusa
        spdlog::spdlog
)

target_include_directories(Atlas PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

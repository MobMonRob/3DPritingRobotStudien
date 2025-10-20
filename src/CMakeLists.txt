add_subdirectory(kronos)
add_subdirectory(medusa)
add_subdirectory(atlas)

add_executable(${PROJECT_NAME}
        main.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
        spdlog::spdlog
        Medusa
        Kronos
        Atlas
)
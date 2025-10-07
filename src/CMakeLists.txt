add_executable(${PROJECT_NAME}
        main.cpp
)

add_subdirectory(kronos)
add_subdirectory(medusa)

target_link_libraries(${PROJECT_NAME} PRIVATE
        medusa
        kronos
)
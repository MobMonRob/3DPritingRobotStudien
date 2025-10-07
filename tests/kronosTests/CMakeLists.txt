add_executable(
        KronosTestDummy
        KronosTestDummy.cpp
)

target_link_libraries(
        KronosTestDummy
        Kronos
)

add_test(NAME KronosTestDummy COMMAND KronosTestDummy)

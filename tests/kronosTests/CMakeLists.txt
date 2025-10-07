add_executable(
        kronosTestDummy
        kronosTestDummy.cpp
)

target_link_libraries(
        kronosTestDummy
        kronos
)

add_test(NAME kronosTestDummy COMMAND kronosTestDummy)

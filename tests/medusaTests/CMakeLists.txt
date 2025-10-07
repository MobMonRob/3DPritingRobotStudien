add_executable(
        medusaTestDummy
        medusaTestDummy.cpp
)

target_link_libraries(
        medusaTestDummy
        medusa
)

add_test(NAME medusaTestDummy COMMAND medusaTestDummy)
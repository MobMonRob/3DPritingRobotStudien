add_executable(
        MedusaTestDummy
        MedusaTestDummy.cpp
)

target_link_libraries(
        MedusaTestDummy
        Medusa
)

add_test(NAME MedusaTestDummy COMMAND MedusaTestDummy)
set(MEDUSA_LOG_CATEGORY "TEST")

medusa_log("Configuring tests")
medusa_log("Subdirs: unit, integration" , VERBOSE)

include(CTest)

add_subdirectory(unit)
add_subdirectory(integration)
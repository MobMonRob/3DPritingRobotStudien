set(MEDUSA_LOG_CATEGORY "BUILD")

medusa_log("Configuring src/ targets")
medusa_log("Subdirs: core, graphics, ui (+ optional app)" , VERBOSE)

add_subdirectory(core)
add_subdirectory(graphics)
add_subdirectory(controls)
add_subdirectory(slicing)
add_subdirectory(ui)

if (MEDUSA_BUILD_APP)
    medusa_log("App enabled -> add_subdirectory(app)")
    add_subdirectory(app)
else ()
    medusa_log("App disabled (MEDUSA_BUILD_APP=OFF)" , NOTICE)
endif ()

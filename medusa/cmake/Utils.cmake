# ==============================================================================
# cmake/Utils.cmake
# Global helper functions for Medusa
# ==============================================================================

# ------------------------------------------------------------------------------
# Function: medusa_log
# Usage:    medusa_log("My message" [level])
#
# Arguments:
#   msg   : The message to print.
#   level : (Optional) CMake log level (STATUS, WARNING, FATAL_ERROR, VERBOSE).
#           Default is STATUS.
#
# Context:
#   Uses the variable MEDUSA_LOG_CATEGORY to determine the log tag.
#   Default category is "GENERAL" if variable is not set.
# ------------------------------------------------------------------------------
function(medusa_log msg)
    # 1) Read level (default: STATUS)
    set(log_level STATUS)
    if (ARGC GREATER 1)
        set(log_level ${ARGV1})
    endif ()

    # 2) Read category from variable (default: GENERAL)
    if (DEFINED MEDUSA_LOG_CATEGORY)
        set(category "${MEDUSA_LOG_CATEGORY}")
    else ()
        set(category "GENERAL")
    endif ()
    string(TOUPPER "${category}" category_upper)

    # 3) Colors
    # We only colorize if it's not a plain STATUS message.
    set(ColorStart "")
    set(ColorEnd "")

    if (NOT "${log_level}" STREQUAL "STATUS")
        # Simple check: ANSI codes on Unix, or if Windows Terminal is active
        if (NOT WIN32 OR DEFINED ENV{WT_SESSION})
            string(ASCII 27 Esc)
            set(ColorEnd "${Esc}[m")

            if ("${log_level}" STREQUAL "FATAL_ERROR" OR "${log_level}" STREQUAL "SEND_ERROR")
                set(ColorStart "${Esc}[31m") # red (error)
            elseif ("${log_level}" STREQUAL "WARNING")
                set(ColorStart "${Esc}[33m") # yellow (warning)
            elseif ("${log_level}" STREQUAL "VERBOSE" OR "${log_level}" STREQUAL "DEBUG")
                set(ColorStart "${Esc}[34m") # blue (info/debug)
            endif ()
        endif ()
    endif ()

    # 4) Output
    # We pass ${log_level} through to message(), so CMake can control behavior (e.g. stopping on FATAL_ERROR)
    message(${log_level} "${ColorStart}[Medusa] [${category_upper}] ${msg}${ColorEnd}")

endfunction()
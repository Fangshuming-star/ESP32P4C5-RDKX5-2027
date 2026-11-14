cmake_minimum_required(VERSION 3.16)

# ------------------------------------------------------------
# Keep tools/p4_tool.py "WiFi config" effective.
# The GUI only rewrites CONFIG_ESP_WIFI_SSID / CONFIG_ESP_WIFI_PASSWORD in
# sdkconfig.defaults. Once sdkconfig exists it overrides those defaults, so we
# copy the two values into sdkconfig before project() runs. The file is only
# written when a value really changed (no cmake reconfigure loop).
# ------------------------------------------------------------
set(_defs "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.defaults")
set(_conf "${CMAKE_CURRENT_LIST_DIR}/sdkconfig")
set_property(DIRECTORY "${CMAKE_CURRENT_LIST_DIR}" APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_defs}")

if(EXISTS "${_defs}" AND EXISTS "${_conf}")
    file(STRINGS "${_defs}" _def_lines)
    foreach(_key CONFIG_ESP_WIFI_SSID CONFIG_ESP_WIFI_PASSWORD)
        set(_want "")
        set(_found FALSE)
        foreach(_line IN LISTS _def_lines)
            if(_line MATCHES "^${_key}=(.*)$")
                string(STRIP "${CMAKE_MATCH_1}" _want)
                set(_found TRUE)
            endif()
        endforeach()
        if(NOT _found)
            continue()
        endif()

        file(STRINGS "${_conf}" _conf_lines)
        set(_have "")
        set(_has_key FALSE)
        foreach(_line IN LISTS _conf_lines)
            if(_line MATCHES "^${_key}=(.*)$")
                string(STRIP "${CMAKE_MATCH_1}" _have)
                set(_has_key TRUE)
            endif()
        endforeach()
        if(_has_key AND _have STREQUAL _want)
            continue()
        endif()

        set(_out "")
        set(_done FALSE)
        foreach(_line IN LISTS _conf_lines)
            if(_line MATCHES "^${_key}=")
                if(NOT _done)
                    list(APPEND _out "${_key}=${_want}")
                    set(_done TRUE)
                endif()
            else()
                list(APPEND _out "${_line}")
            endif()
        endforeach()
        if(NOT _done)
            list(APPEND _out "${_key}=${_want}")
        endif()
        string(REPLACE ";" "\n" _text "${_out}")
        file(WRITE "${_conf}" "${_text}\n")
        message(STATUS "wifi-sync: ${_key} updated from sdkconfig.defaults -> ${_want}")
    endforeach()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(DIpances)

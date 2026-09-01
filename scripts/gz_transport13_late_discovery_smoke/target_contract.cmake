# ``gz-transport13::core`` is an INTERFACE_IMPORTED convenience target in the
# 13.5.0 config. The frozen shared object belongs to the concrete target.
if(NOT TARGET gz-transport13::core)
  message(FATAL_ERROR "gz-transport13::core target is missing")
endif()
if(NOT TARGET gz-transport13::gz-transport13)
  message(FATAL_ERROR "gz-transport13 concrete shared target is missing")
endif()

get_target_property(TZCUP_CORE_INTERFACE_LINKS gz-transport13::core
  INTERFACE_LINK_LIBRARIES)
if(NOT TZCUP_CORE_INTERFACE_LINKS)
  message(FATAL_ERROR "gz-transport13::core has no interface transport target")
endif()
set(TZCUP_CORE_TRANSPORT_TARGETS "")
foreach(TZCUP_CORE_LINK IN LISTS TZCUP_CORE_INTERFACE_LINKS)
  if(TZCUP_CORE_LINK MATCHES "^gz-transport13::")
    list(APPEND TZCUP_CORE_TRANSPORT_TARGETS "${TZCUP_CORE_LINK}")
  endif()
endforeach()
list(LENGTH TZCUP_CORE_TRANSPORT_TARGETS TZCUP_CORE_TRANSPORT_TARGET_COUNT)
if(NOT TZCUP_CORE_TRANSPORT_TARGET_COUNT EQUAL 1 OR
   NOT TZCUP_CORE_TRANSPORT_TARGETS STREQUAL "gz-transport13::gz-transport13")
  message(FATAL_ERROR
    "gz-transport13::core must link only the concrete frozen shared target: ${TZCUP_CORE_TRANSPORT_TARGETS}")
endif()

set(TZCUP_TRANSPORT_IMPORTED_LOCATION "")
foreach(TZCUP_LOCATION_PROPERTY
    IMPORTED_LOCATION_RELEASE IMPORTED_LOCATION_NOCONFIG IMPORTED_LOCATION)
  get_target_property(TZCUP_CANDIDATE_LOCATION gz-transport13::gz-transport13
    ${TZCUP_LOCATION_PROPERTY})
  if(TZCUP_CANDIDATE_LOCATION AND
     NOT TZCUP_CANDIDATE_LOCATION MATCHES "-NOTFOUND$")
    set(TZCUP_TRANSPORT_IMPORTED_LOCATION "${TZCUP_CANDIDATE_LOCATION}")
    break()
  endif()
endforeach()
if(NOT TZCUP_TRANSPORT_IMPORTED_LOCATION)
  message(FATAL_ERROR
    "gz-transport13 concrete shared target has no imported library location")
endif()

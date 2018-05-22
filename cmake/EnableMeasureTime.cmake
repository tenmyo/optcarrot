option(MEASURETIME_ENABLE
  "Measure time of compile."
  ON)
if(MEASURETIME_ENABLE)
  message(STATUS "Enable measure time of compile")
  set_property(GLOBAL APPEND PROPERTY RULE_LAUNCH_COMPILE "${CMAKE_COMMAND} -E time")
  set_property(GLOBAL APPEND PROPERTY RULE_LAUNCH_LINK "${CMAKE_COMMAND} -E time")
endif()

set(controller_SRC MoveItFSM.cpp)

set(controller_HDR MoveItFSM.h)

add_library(${PROJECT_NAME}_fsm SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME}_fsm PROPERTIES COMPILE_FLAGS
                                                     "-DMoveItFSM_EXPORTS")
target_link_libraries(${PROJECT_NAME}_fsm PUBLIC mc_rtc::mc_control_fsm)
install(
  TARGETS ${PROJECT_NAME}_fsm
  ARCHIVE DESTINATION "${MC_RTC_LIBDIR}"
  LIBRARY DESTINATION "${MC_RTC_LIBDIR}"
  RUNTIME DESTINATION bin)

add_controller(${PROJECT_NAME}_fsm_controller lib.cpp "")
target_link_libraries(${PROJECT_NAME}_fsm_controller PUBLIC ${PROJECT_NAME}_fsm)

add_subdirectory(states)

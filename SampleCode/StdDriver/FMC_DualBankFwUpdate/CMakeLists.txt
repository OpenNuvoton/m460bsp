create_project(App ${CMAKE_CURRENT_LIST_DIR}/App/main.c)
target_include_directories(App PUBLIC ${CMAKE_CURRENT_LIST_DIR}/common/inc ${BSP_DIR}/Library/NuMaker/xmodem)
target_link_libraries(App xmodem_lib)

create_project(Loader ${CMAKE_CURRENT_LIST_DIR}/Loader/main.c)
target_include_directories(Loader PUBLIC ${CMAKE_CURRENT_LIST_DIR}/common/inc)

# Drone Vision External Code

add_library ( DroneVisionExt fast9/fastRosten.c)

target_include_directories ( DroneVisionExt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

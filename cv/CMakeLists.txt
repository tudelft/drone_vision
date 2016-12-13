# Drone Vision

add_library ( DroneVision image.c streaming/rtp.c streaming/udp_socket.c)
# encoding/rtp.c)

set(CMAKE_C_FLAGS "-std=gnu99") 

target_include_directories ( DroneVision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

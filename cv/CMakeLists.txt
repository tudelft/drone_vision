# Drone Vision

add_library ( DroneVision std.h image.h image.c streaming/rtp.c streaming/udp_socket.c encoding/jpeg.c)
# encoding/rtp.c)

set(CMAKE_C_FLAGS "-std=gnu99") 

target_include_directories ( DroneVision PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

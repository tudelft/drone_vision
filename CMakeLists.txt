# Drone Vision
cmake_minimum_required (VERSION 2.6) 

project ( DroneVision ) 

if(WIN32)
   if(MSVC)
      add_definitions(-DWIN32_LEAN_AND_MEAN)
      add_definitions(-D_CRT_SECURE_NO_WARNINGS)
   endif(MSVC)
else()
#   add_definitions(-Wno-write-strings)
endif(WIN32)

set ( EXECUTABLE_OUTPUT_PATH "${DroneVision_SOURCE_DIR}/bin" )



# Library
add_subdirectory (cv)

# External Code
add_subdirectory (ext)

# Test Code
add_subdirectory (test)

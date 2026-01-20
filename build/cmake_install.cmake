# Install script for directory: /home/lxj/Desktop/Parking/J6B_Parking_PNC

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libcommon.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libgrid_map.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libparking_space_evaluation.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libparking_space_recommendation.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libhybrid_astar.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libtrajectory_smoothing.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libvelocity_planning.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libtrajectory_merging.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libtrajectory_optimization.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libtrajectory_output.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libcore.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/lib/libj6b_parking_pnc.a")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/common/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/grid_map/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/parking_space_evaluation/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/parking_sapce_recommendation/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/hybrid_astar/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/trajectory_smoothing/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/velocity_planning/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/trajectory_merging/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/trajectory_optimization/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/trajectory_output/cmake_install.cmake")
  include("/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/src/core/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/lxj/Desktop/Parking/J6B_Parking_PNC/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

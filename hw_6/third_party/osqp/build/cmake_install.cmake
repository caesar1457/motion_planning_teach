# Install script for directory: /home/caesar/motion_planning_ws/src/hw_6/third_party/osqp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/out/libosqp.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/osqp" TYPE FILE FILES
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/auxil.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/constants.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/error.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/glob_opts.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/lin_alg.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/osqp.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/osqp_configure.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/proj.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/scaling.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/types.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/util.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/cs.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/polish.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/lin_sys.h"
    "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/include/ctrlc.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/out/libosqp.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libosqp.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake"
         "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/CMakeFiles/Export/lib/cmake/osqp/osqp-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp/osqp-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/CMakeFiles/Export/lib/cmake/osqp/osqp-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/CMakeFiles/Export/lib/cmake/osqp/osqp-targets-noconfig.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/osqp" TYPE FILE FILES "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/osqp-config.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/src/cmake_install.cmake")
  include("/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/include/cmake_install.cmake")
  include("/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/lin_sys/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/caesar/motion_planning_ws/src/hw_6/third_party/osqp/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

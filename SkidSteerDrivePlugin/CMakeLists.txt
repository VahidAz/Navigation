cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#//////////////////////////////////////////////////////////***Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#//////////////////////////////////////////////////////////***Protobuf
find_package(Protobuf REQUIRED)

#//////////////////////////////////////////////////////////***Set
set (plugins
  #CameraPlugin
  #DepthCameraPlugin
  #GpuRayPlugin
  #ContactPlugin
  #CartTestPlugin
  #ForceTorquePlugin
  #ModelTrajectoryTestPlugin
  #RayPlugin
  #RubblePlugin
  #SpringTestPlugin
  #DiffDrivePlugin
  SkidSteerDrivePlugin
  #JointTrajectoryPlugin
  #VehiclePlugin
)

#//////////////////////////////////////////////////////////***Build Plugin
foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} ${GAZEBO_LIBRARIES})
  install (TARGETS ${src} DESTINATION /usr/lib/gazebo-1.8/plugins/)
  #gz_install_includes("plugins" ${src}.hh)
endforeach (src ${plugins})

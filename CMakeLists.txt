cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#//////////////////////////////////////////////////////////***QT4
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

#//////////////////////////////////////////////////////////***Gazebo
include(FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

#//////////////////////////////////////////////////////////***Boost
include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

#//////////////////////////////////////////////////////////***Protobuf
find_package(Protobuf REQUIRED)

#//////////////////////////////////////////////////////////***FuzzyLite (Fuzzy Library)
include_directories(. ${CMAKE_SOURCE_DIR}/../Fuzzylite)

#//////////////////////////////////////////////////////////***LA (Learning Automata)
add_library(LA LA.cc)
target_link_libraries(LA)

#//////////////////////////////////////////////////////////***Fuzzy (Navigation)
add_library(Fuzzy Fuzzy.cc)
target_link_libraries(Fuzzy fuzzylite)

#//////////////////////////////////////////////////////////***Manage
add_library(Manage Manage.cc)
target_link_libraries(Manage ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} LA Fuzzy)

#//////////////////////////////////////////////////////////***main
add_executable(main  main.cc)
target_link_libraries(main Manage ${QT_LIBRARIES})

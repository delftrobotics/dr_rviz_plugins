cmake_minimum_required(VERSION 2.8.3)
project(dr_rviz_plugins)

find_package(catkin REQUIRED COMPONENTS
	class_loader
	rviz
	roscpp
	tf
)

find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED)

pkg_check_modules(OGRE OGRE)
link_directories(${OGRE_LIBRARY_DIRS})

option(UseQt5 "UseQt5" OFF)
if (UseQt5)
	find_package(Qt5Core    REQUIRED)
	find_package(Qt5Widgets REQUIRED)
	set(Qt_INCLUDE_DIRS "")
	set(Qt_LIBRARIES    Qt5::Core Qt5::Widgets)
else()
	find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
	set(Qt_INCLUDE_DIRS ${Qt4_INCLUDE_DIRS})
	set(Qt_LIBRARIES    ${Qt4_LIBRARIES})
	include(${QT_USE_FILE})
endif()
add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ON)

set(INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${Qt_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
set(LIBRARIES    ${catkin_LIBRARIES}    ${Qt_LIBRARIES}    ${Boost_LIBRARIES})


catkin_package(
	LIBRARIES ${PROJECT_NAME}
)

include_directories(
	include/${PROJECT_NAME}
	SYSTEM ${INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
	src/pose_array.cpp
)

target_link_libraries(${PROJECT_NAME} ${LIBRARIES})

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_descriptions.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

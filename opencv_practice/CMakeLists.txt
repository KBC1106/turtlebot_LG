cmake_minimum_required(VERSION 2.8.3)
project(opencv_practice)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
  DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


add_executable(opencv_ros2 src/opencv_ros2.cpp)
target_link_libraries(opencv_ros2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(check_photo src/check_photo.cpp)
target_link_libraries(check_photo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(position_get src/position_get.cpp)
target_link_libraries(position_get ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})





#############
## Install ##
#############
foreach(dir launch img rviz)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)


#############
## Testing ##
#############

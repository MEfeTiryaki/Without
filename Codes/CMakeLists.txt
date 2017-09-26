cmake_minimum_required(VERSION 2.8.3)
project(without)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation

)
find_package( OpenCV REQUIRED ) ### THIS IS IMPORTANTTTTT<-----------

 add_message_files(
   DIRECTORY msg
   FILES
   flatten2DArray.msg
   Map.msg
   Vector4.msg
   Player.msg
   Sensor.msg
 )
 add_service_files(
    DIRECTORY srv
    FILES
    EyeData.srv
    MasterControl.srv
    PlayerControl.srv
    VirtualEngCont.srv
    RobotControl.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES without
#  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(Eye src/Eye.cpp)
target_link_libraries(Eye ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(Eye without_gencpp)


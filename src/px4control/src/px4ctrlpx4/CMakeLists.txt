cmake_minimum_required(VERSION 3.0.2)
project(px4ctrlpx4)

# 1. 分开查找catkin和JSONCPP（不能混在同一个find_package中）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  sensor_msgs
)


# 3. 必须包含Boost（如果需要）
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}  # 如果生成库需要添加
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs quadrotor_msgs sensor_msgs
)

# 4. 包含所有必要的目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 5. 添加可执行文件
add_executable(px4ctrlpx4 
  src/px4ctrlpx4.cpp 
  src/socket_receiver.cpp
)

# 6. 正确链接所有库
target_link_libraries(px4ctrlpx4
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
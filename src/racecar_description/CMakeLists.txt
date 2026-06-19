cmake_minimum_required(VERSION 3.8)
project(racecar_description)

find_package(ament_cmake REQUIRED)

# 这是个纯资源包（无 C++ 代码）：把这几个目录装进 share/，
# 让 get_package_share_directory() 和 ros2 launch 能找到它们。
install(
  DIRECTORY urdf worlds launch tracks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

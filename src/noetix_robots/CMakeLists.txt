cmake_minimum_required(VERSION 3.8)
project(noetix_robots)

find_package(ament_cmake REQUIRED)

# Every robot directory is installed under:
#   <prefix>/share/noetix_robots/<robot_name>/...
#
# Launch/runtime code resolves this tree through ament_index.
foreach(robot_dir common n3 bumi3 bumi4340 bumi2)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${robot_dir}")
    install(
      DIRECTORY ${robot_dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_package()

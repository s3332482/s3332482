cmake_minimum_required(VERSION 3.8)
project(krytn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

install(PROGRAMS
   krytn/app.py
   krytn/move_krytn_action.py
   krytn/move_krytn_topic.py
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
   krytn/static/
   DESTINATION lib/${PROJECT_NAME}/static
)

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
    robot_description
)


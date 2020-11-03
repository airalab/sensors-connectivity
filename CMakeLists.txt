cmake_minimum_required(VERSION 2.8.3)
project(sensors_connectivity)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/worker_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/agent.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
    FILES
        config/default.json
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

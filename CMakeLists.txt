cmake_minimum_required(VERSION 3.0.2)

project(vosk_asr)

find_package(catkin)

catkin_python_setup()
catkin_package()

install(PROGRAMS nodes/asr
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        launch/en-us.launch
        launch/es.launch
        launch/small-en-us.launch
        launch/small-es.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# this project layout and configuration is based on:
# https://www.artificialhumancompanions.com/structure-python-based-ros-package/
# https://github.com/SimonBirrell/simontest

cmake_minimum_required(VERSION 2.8.3)
project(tesseract_robotraconteur)

set (CMAKE_CXX_STANDARD 11)

find_package(tesseract_environment REQUIRED)
find_package(tesseract_kinematics REQUIRED)
find_package(tesseract_visualization REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_time_parameterization REQUIRED)
#find_package(tesseract_rosutils REQUIRED)
find_package(trajopt REQUIRED)
find_package(trajopt_utils REQUIRED)
find_package(trajopt_sco REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(Bullet REQUIRED)
find_package(RobotRaconteur REQUIRED)
find_package(RobotRaconteurCompanion REQUIRED)
find_package(assimp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost COMPONENTS date_time filesystem system regex chrono atomic thread random program_options locale REQUIRED)
link_directories(${ASSIMP_LIBRARY_DIRS})

include_directories(${RobotRaconteur_INCLUDE_DIR} 
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${tesseract_INCLUDE_DIRS}
                    ${tesseract_environment_INCLUDE_DIRS}
                    ${tesseract_kinematics_INCLUDE_DIRS}
                    ${tesseract_visualization_INCLUDE_DIRS}
                    ${tesseract_motion_planners_INCLUDE_DIRS}
                    ${tesseract_time_parameterization_INCLUDE_DIRS}
                    ${BULLET_INCLUDE_DIRS}
                    ${trajopt_INCLUDE_DIRS}
                    ${trajopt_utils_INCLUDE_DIRS}
                    ${trajopt_sco_INCLUDE_DIRS}
                    ${YAML_CPP_INCLUDE_DIR}
                    )

add_executable(tesseract_robotraconteur_service 
    src/tesseract_robotraconteur_service.cpp
    src/tesseract_planner_impl.h
    src/tesseract_planner_impl.cpp
    ${RR_THUNK_HDRS}
    ${RR_THUNK_SRCS}
)

target_link_libraries(tesseract_robotraconteur_service 
    RobotRaconteurCore
    RobotRaconteurCompanion
    tesseract::tesseract_motion_planners_core
    tesseract::tesseract_motion_planners_trajopt
    tesseract::tesseract_motion_planners_simple
    tesseract::tesseract_time_parameterization
    tesseract::tesseract_environment_core
    tesseract::tesseract_environment_kdl
    trajopt::trajopt_sco
    jsoncpp_lib
    yaml-cpp
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}    
    ${RobotRaconteurCore_Extra_Libraries}
)

include(GNUInstallDirs)

install(TARGETS tesseract_robotraconteur_service
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
robotraconteur_generate_thunk(
    RR_THUNK_SRCS
    RR_THUNK_HDRS
    experimental.tesseract_robotics.common.robdef
    experimental.tesseract_robotics.command_language.robdef
    experimental.tesseract_robotics.environment.robdef
    experimental.tesseract_robotics.environment.commands.robdef
    experimental.tesseract_robotics.scene_graph.robdef
    experimental.tesseract_robotics.tasks.robdef
    experimental.tesseract_robotics.robdef
    MASTER_HEADER
    INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../robdef
    ${RobotRaconteur_STANDARD_ROBDEF_DIRS}
    AUTO_IMPORT)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(freespace_example ${RR_THUNK_SRCS} ${RR_THUNK_HDRS} freespace_move_example.cpp)

target_link_libraries(freespace_example RobotRaconteurCompanion RobotRaconteurCore Eigen3::Eigen)

set(SRC_LIST
    General6S.cpp
    ArcInterp.cpp
    Scara4S.cpp
    Interpolation.cpp
    JointInterp.cpp
    SerialRobotModel.cpp
    Controller.cpp
    LineInterp.cpp
    Robot.cpp
    )
#add_library(libRobot ${SRC_LIST})
add_library(libRobot SHARED ${SRC_LIST})
set_target_properties(libRobot PROPERTIES OUTPUT_NAME "Robot")

rock_library(motion_controller
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(motion_controller_bin Main.cpp
    DEPS motion_controller)


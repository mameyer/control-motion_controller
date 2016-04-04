rock_library(motion_controller
    SOURCES
            Controller.cpp
            Ackermann.cpp
    HEADERS MotionControllerTypes.hpp
            Controller.hpp
            Ackermann.hpp
    DEPS_PKGCONFIG
            base-types
            trajectory_follower
    )
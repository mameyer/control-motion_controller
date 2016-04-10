rock_library(motion_controller
    SOURCES
            Controller.cpp
            Ackermann.cpp
            Lateral.cpp
            PointTurn.cpp
    HEADERS MotionControllerTypes.hpp
            Controller.hpp
            Ackermann.hpp
            Lateral.hpp
            PointTurn.hpp
    DEPS_PKGCONFIG
            base-types
            trajectory_follower
    )
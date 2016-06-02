rock_library(motion_controller
    SOURCES
            Controller.cpp
            Ackermann.cpp
            Lateral.cpp
            Dispatcher.cpp
    HEADERS MotionControllerTypes.hpp
            Controller.hpp
            Ackermann.hpp
            Lateral.hpp
            Dispatcher.hpp
    DEPS_PKGCONFIG
            base-types
            trajectory_follower
    )
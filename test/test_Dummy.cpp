#include <boost/test/unit_test.hpp>
#include <motion_controller/Dummy.hpp>

using namespace motion_controller;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    motion_controller::DummyClass dummy;
    dummy.welcome();
}

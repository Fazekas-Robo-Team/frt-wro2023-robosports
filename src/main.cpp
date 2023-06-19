#include "lib.hpp"

[[noreturn]] void left_main ()
{
    using namespace FRT;
    using namespace FRT::unit_literals;

    left_wheel.set_duty_cycle_setpoint(100);
    right_wheel.set_duty_cycle_setpoint(100);
    left_wheel.run_command(TachoMotor::commands::run_direct);
    right_wheel.run_command(TachoMotor::commands::run_direct);

    while (true) {
        Logger::info(left_wheel.get_state());
    }
}

[[noreturn]] void right_main ()
{
    FRT::Logger::error("Not implemented.");
    exit(EXIT_FAILURE);
}

int main () 
{
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    #if FRT_ROBOT_ID == 0
    left_main();
    #else
    right_main();
    #endif

    FRT::Logger::error("Control loop exited unexpectedly.");
}

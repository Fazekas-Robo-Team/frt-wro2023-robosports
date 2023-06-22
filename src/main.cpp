#include "lib.hpp"

using namespace FRT;
using namespace FRT::unit_literals;



void move (const Unit auto segment)
{
    int segment_pulses = left_wheel.units_to_pulses(segment);
    int direction = (segment_pulses > 0) ? 1 : -1;

    segment_pulses *= direction;

    left_wheel.set_duty_cycle_setpoint(0);
    right_wheel.set_duty_cycle_setpoint(0);

    left_wheel.run_command(TachoMotor::commands::run_direct);
    right_wheel.run_command(TachoMotor::commands::run_direct);

    left_wheel.set_stop_action(TachoMotor::stop_actions::brake);
    right_wheel.set_stop_action(TachoMotor::stop_actions::brake);

    const double left_start = left_wheel.get_position<deg>().value * direction;
    const double right_start = right_wheel.get_position<deg>().value * direction;

    bool left_on = true, right_on = true;

    const double target_speed = 1050;
    const double Kp = (direction > 0) ? 0.001 : 0.0001;
    const double Ki = 0.00001;
    const double Kd = 0.00002;
    // high baseline sp, will not start without it
    double sp = 40, last_error = 0, error_sum = 0;

    while (true) {
        const double left_pos = (direction * left_wheel.get_position<deg>().value) - left_start;
        const double right_pos = (direction * right_wheel.get_position<deg>().value) - right_start;

        if (left_on && left_pos > segment_pulses) {
            left_wheel.stop();
            left_on = false;
        }

        if (right_on && right_pos > segment_pulses) {
            right_wheel.stop();
            right_on = false;
        }

        if (!left_on && !right_on) {
            break;
        }

        const double speed = (left_wheel.get_speed<deg>().value + right_wheel.get_speed<deg>().value) / 2 * direction;
        const double speed_error = speed - target_speed;

        const double correction = speed_error * Kp + error_sum * Ki + (speed_error - last_error) * Kd;
        sp = clamp(sp - correction, -100.0, 100.0);
        last_error = speed_error;
        error_sum += speed_error;

        const double diff = (left_pos - right_pos) * 5;
        const double left_sp = (left_pos > right_pos) ? sp - diff : sp;
        const double right_sp = (right_pos > left_pos) ? sp + diff : sp;

        if (left_on) {
            left_wheel.set_duty_cycle_setpoint(left_sp * direction);
        }

        if (right_on) {
            right_wheel.set_duty_cycle_setpoint(right_sp * direction);
        }

        Logger::info(sp, target_speed, speed, left_pos, right_pos, left_sp, right_sp);
    }
}

[[noreturn]] void left_main ()
{
    Sound::beep(1000, 100);

    left_wheel.set_polarity(TachoMotor::polarities::inversed);
    right_wheel.set_polarity(TachoMotor::polarities::inversed);
    move(2m);
    for (;;) {
        move(115cm);
        sleep(200ms);
        move(-115cm);
        sleep(200ms);
    }

    exit(EXIT_SUCCESS);
}

[[noreturn]] void right_main ()
{
    Logger::error("Not implemented.");
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

    Logger::error("Control loop exited unexpectedly.");
}

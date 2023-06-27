#include "lib.hpp"

/*
TODO:
- wallbang based on speed
- wallbang turn
*/

inline void clearing_corner ()
{
    move_segment(4cm, 0deg);
    turn(-90deg);
    move_wallbang(52cm, -90deg);
    move_segment(-4cm, -90deg);
    turn(12deg);
    unregulated_move(-100, 1000ms);
}

[[noreturn]] void left_main ()
{
    left_wheel.set_polarity(TachoMotor::polarities::inversed);
    right_wheel.set_polarity(TachoMotor::polarities::inversed);

    unregulated_move(-100, 400ms);

    gyro.reset();

    move_wallbang(114cm, 0deg);
    unregulated_move(70, 200ms);
    lift_up();
    unregulated_move(70, 50ms);
    unregulated_move(-70, 50ms);
    turn(-10deg);
    lift_down();
    move_wallbang(-127cm, -13deg);
    unregulated_move(-100, 400ms);

    move_wallbang(114cm, 0deg);
    unregulated_move(70, 200ms);
    lift_up();
    unregulated_move(70, 50ms);
    lift_down();
    unregulated_move(-70, 50ms);
    move_wallbang(-127cm, 0deg);
    unregulated_move(-100, 400ms);

    while (true) {
        clearing_corner();
        gyro.reset();

        move_wallbang(114cm, -1.5deg);
        unregulated_move(70, 200ms);
        lift_up();
        unregulated_move(70, 50ms);
        unregulated_move(-70, 50ms);
        lift_down();
        move_wallbang(-130cm, -10deg);
        unregulated_move(-100, 400ms);

        move_wallbang(114cm, 0deg);
        unregulated_move(70, 200ms);
        lift_up();
        unregulated_move(70, 50ms);
        unregulated_move(-70, 50ms);
        lift_down();
        move_wallbang(-127cm, -10deg);
        unregulated_move(-100, 400ms);

        move_wallbang(114cm, 2deg);
        unregulated_move(70, 200ms);
        lift_up();
        unregulated_move(70, 50ms);
        unregulated_move(-70, 50ms);
        lift_down();
        move_wallbang(-127cm, 0deg);
        unregulated_move(-100, 400ms);
    }

    exit(EXIT_SUCCESS);
}

inline void shoot_arm ()
{
    arm.set_duty_cycle_setpoint(-100);
    
}

inline void collect_arm()
{
    arm.set_duty_cycle_setpoint(40);
}

[[noreturn]] void right_main ()
{
    collect_arm();
    arm.run_command(TachoMotor::commands::run_direct);

    left_wheel.set_polarity(TachoMotor::polarities::normal);
    right_wheel.set_polarity(TachoMotor::polarities::normal);

    unregulated_move(-100, 400ms);

    gyro.reset();

    arm.set_duty_cycle_setpoint(-100);
    arm.run_command(TachoMotor::commands::run_direct);

    move_segment(68.5cm, 0deg);
    move_segment(-5cm, 0deg);
    turn(-17deg);
    shoot_arm();
    move_wallbang(-77cm, -20deg);
    collect_arm();

    unregulated_move(-100, 400ms);

    move_segment(68.5cm, 0deg);
    move_segment(-5cm, 0deg);
    // bacc
    shoot_arm();
    turn(20deg);
    move_wallbang(-77cm, 25deg);
    collect_arm();

    turn(0deg);
    while (true) {
        unregulated_move(-100, 400ms);
        move_segment(3.5cm, 0deg);
        turn(90deg);
        move_segment(-15cm, 90deg);
        move_wallbang(65cm, 90deg);
        move_segment(-7cm, 90deg);
        turn(-25deg);
        unregulated_move(-100, 600ms);

        move_wallbang(68.5cm, 1deg);
        shoot_arm();
        move_wallbang(-75cm, 8.5deg);
        collect_arm();
        unregulated_move(-100, 400ms);

        gyro.reset();
        Logger::info(gyro.get_angle(), gyro.base);


        move_wallbang(68.5cm, 0deg);
        shoot_arm();
        move_wallbang(-75cm, 8.5deg);
        collect_arm();
        unregulated_move(-100, 400ms);

        move_wallbang(68.5cm, -2deg);
        shoot_arm();
        move_wallbang(-75cm, 0deg);
        collect_arm();
    }

    exit(EXIT_SUCCESS);
}

int main () 
{
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    gyro.set_mode(GyroSensor::modes::calibration);
    sleep(150ms);
    gyro.set_mode(GyroSensor::modes::angle_and_rate);
    sleep(150ms);

    #if FRT_ROBOT_ID == 0
    left_main();
    #else
    right_main();
    #endif

    Logger::error("Control loop exited unexpectedly.");
}

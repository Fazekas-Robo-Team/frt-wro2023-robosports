#pragma once

#include <frt/frt.hpp>

using namespace FRT;
using namespace FRT::unit_literals;

#if !defined FRT_ROBOT_ID || (FRT_ROBOT_ID != 0) && (FRT_ROBOT_ID != 1)
#error FRT_ROBOT_ID not defined.
#endif

GyroSensor gyro(INPUT_1);

#if FRT_ROBOT_ID == 0

// 5 cm wheels with a gear ratio of 3
TachoMotor left_wheel {OUTPUT_B, cm(5 * 3.0)};
TachoMotor right_wheel {OUTPUT_C, cm(5 * 3.0)};
// gear has 12 teeth, rack has one tooth per 3.2mm
TachoMotor arm {OUTPUT_A, mm(12.0 * 3.2 / M_PI)};

#else

TachoMotor left_wheel {OUTPUT_C, cm(5 * (36.0 / 20.0))};
TachoMotor right_wheel {OUTPUT_B, cm(5 * (36.0 / 20.0))};
TachoMotor arm (OUTPUT_A, cm(0));

#endif

struct MoveState
{
    double position;
    double speed;
    double dir_error;
};

struct MoveControl
{
    #if FRT_ROBOT_ID == 0
    const double Kp = 0.0008;
    const double Ki = 0;
    const double Kd = 0.00002;

    const double Dp = 3;
    const double Di = 0.01;
    const double Dd = 0.5;
    #else
    const double Kp = 0.0012;
    const double Ki = 0;
    const double Kd = 0.00002;

    const double Dp = 3;
    const double Di = 0.01;
    const double Dd = 0.5;
    #endif
};

struct SegmentControl : public MoveControl
{
    int segment_pulses;

    SegmentControl (const Unit auto segment)
    {
        segment_pulses = left_wheel.units_to_pulses(segment);
    }

    bool exit_condition (const MoveState &state)
    {
        return state.position >= segment_pulses;
    }

    int speed_control (const MoveState &state)
    {
        static const auto end_threshold = left_wheel.units_to_pulses(60cm);
        static const int target_speed = 1050;

        const int end_distance = segment_pulses - state.position;

        if (end_distance < end_threshold) {
            #if FRT_ROBOT_ID == 0
            return target_speed * clamp(0 + (double)end_distance / end_threshold, 0.0, 1.0);
            #else
            return target_speed * clamp(0.2 + (double)end_distance / end_threshold, 0.0, 1.0);
            #endif
        }
        else {
            return target_speed;
        }
    }
};

struct SegmentWallbangControl : public SegmentControl
{
    using SegmentControl::SegmentControl;

    //double last_speed = 0, avg = 0;
    int cycles = 0;
    static const int cycles_threshold = 5;

    bool exit_condition (const MoveState &state)
    {
        if (state.position >= segment_pulses) {
            return true;
        }

        if (left_wheel.is_stalled() || right_wheel.is_stalled()) {
            cycles++;
        } else {
            cycles = 0;
        }

        if (cycles > cycles_threshold) {
            return true;
        }

        return false;

        /* static const auto threshold = left_wheel.units_to_pulses(5cm);
        const double acceleration = state.speed - last_speed;
        avg += (1.0 / 20.0) * (acceleration - avg);
        last_speed = state.speed;
        return state.position > threshold && (avg <= -2 || state.speed <= 0); */
        
    }
};

struct TimerControl : public MoveControl
{
    const double seconds;
    const double start = time();

    TimerControl (const double seconds)
    : seconds(seconds)
    {}

    bool exit_condition ([[maybe_unused]] const MoveState &state)
    {
        return time() > start + seconds;
    }
};

struct TurnControl
{
    #if FRT_ROBOT_ID == 0
    const double Kp = 0;
    const double Ki = 0;
    const double Kd = 0;
    #else
    const double Kp = 0;
    const double Ki = 0;
    const double Kd = 0;
    #endif

    const double Dp = 0.5;
    const double Di = 0.01;
    const double Dd = 0.1;

    int cycles = 0;
    static const int cycles_threshold = 10;

    bool exit_condition (const MoveState &state) 
    {
        if (abs(state.dir_error) <= 2) {
            cycles++;
            if (cycles > cycles_threshold) {
                return true;
            }
        }
        else {
            cycles = 0;
        }
        return false;
    }

    int speed_control ([[maybe_unused]] const MoveState &state) 
    {
        return 0;
    }
};

inline void move (const int direction, const Angle auto target_angle, auto control)
{
    left_wheel.set_duty_cycle_setpoint(0);
    right_wheel.set_duty_cycle_setpoint(0);

    left_wheel.run_direct();
    right_wheel.run_direct();

    left_wheel.set_stop_action(TachoMotor::stop_actions::brake);
    right_wheel.set_stop_action(TachoMotor::stop_actions::brake);

    const double left_start = left_wheel.get_position<deg>().value * direction;
    const double right_start = right_wheel.get_position<deg>().value * direction;

    // high baseline sp, will not start without it
    double sp = 35, last_error = 0, error_sum = 0;

    const double target_deg = angle_cast<deg>(target_angle).value;
    double dir_error_sum = 0;

    // static correction

    #if FRT_ROBOT_ID == 0
    const double left_corr = 1, right_corr = 1;
    #else
    const double left_corr = 1, right_corr = 1;
    #endif

    while (true) {
        const double left_pos = (direction * left_wheel.get_position<deg>().value) - left_start;
        const double right_pos = (direction * right_wheel.get_position<deg>().value) - right_start;

        // exit conditions

        const double position = (left_pos + right_pos) / 2;
        const double speed = (left_wheel.get_speed<deg>().value + right_wheel.get_speed<deg>().value) / 2 * direction;

        const auto gyro_state = gyro.get_angle_and_rate();
        const double dir_error = gyro_state.angle.value - target_deg;

        MoveState state {
            .position = position,
            .speed = speed,
            .dir_error = dir_error
        };

        if (control.exit_condition(state)) {
            right_wheel.stop();
            left_wheel.stop();
            break;
        }

        // speed pidconst double Kp = 0.008, Ki = 0.00000007, Kd = 0.00002;

        const int target_speed = control.speed_control(state);
        //Logger::info(target_speed);
        const double speed_error = speed - target_speed;

        const double correction = speed_error * control.Kp + error_sum * control.Ki + (speed_error - last_error) * control.Kd;
        sp = clamp(sp - correction, -100.0, 100.0);
        last_error = speed_error;
        error_sum += speed_error;

        // direction pid

        const double dir_correction = dir_error * control.Dp + dir_error_sum * control.Di + gyro_state.rate.value * control.Dd;
        dir_error_sum += dir_error;

        // calculating duty cycle setpoint

        const double left_sp = clamp(sp - direction * dir_correction, -100.0, 100.0);
        const double right_sp = clamp(sp + direction * dir_correction, -100.0, 100.0);

        sp = (left_sp + right_sp) / 2;

        // updating motors

        left_wheel.set_duty_cycle_setpoint(left_sp * left_corr * direction);
        right_wheel.set_duty_cycle_setpoint(right_sp * right_corr * direction);
        //Logger::info(dir_error, left_sp * left_corr * direction, right_sp * right_corr * direction);

        Logger::info(dir_error);
    }

    while (left_wheel.get_speed() != 0cm || right_wheel.get_speed() != 0cm) {}
}

inline void turn (const Angle auto target_angle)
{
    left_wheel.set_duty_cycle_setpoint(0);
    right_wheel.set_duty_cycle_setpoint(0);

    left_wheel.run_direct();
    right_wheel.run_direct();

    left_wheel.set_stop_action(TachoMotor::stop_actions::brake);
    right_wheel.set_stop_action(TachoMotor::stop_actions::brake);

    const double dir_end = angle_cast<deg>(target_angle).value;
    const double dir_start = gyro.get_angle().value;
    const int direction = (dir_end - dir_start > 0) ? 1 : -1;

    #if FRT_ROBOT_ID == 0
    const double Kp = 0.008, Ki = 0.00000007, Kd = 0.00002;
    const double sp_limit = 70;
    const double max_speed_target = 200;
    const double base_speed_target = 0.2;
    #else
    const double Kp = 0.008, Ki = 0.0000001, Kd = 0.00003;
    const double sp_limit = 100;
    const double max_speed_target = 320;
    const double base_speed_target = 0.1;
    #endif

    double speed_target = max_speed_target;

    int cycles = 0;
    static const int cycles_threshold = 5;

    double left_sp = 20 * direction, right_sp = -20 * direction, left_sum = 0, left_last = 0, right_sum = 0, right_last = 0;

    while (cycles < cycles_threshold) {
        const double distance = (dir_end - gyro.get_angle().value) * direction;

        if (abs(distance) <= 1) {
            cycles++;
        }
        else {
            cycles = 0;
        }

        if (distance < 60) {
            speed_target = max_speed_target * clamp((distance > 0 ? 1 : -1) * base_speed_target + distance / 60, -1.0, 1.0);
        }

        const double left_speed = left_wheel.get_speed<deg>().value * direction;
        const double right_speed = right_wheel.get_speed<deg>().value * direction;

        const double left_error = left_speed - speed_target;
        const double right_error = right_speed - (-speed_target);

        const double left_correction = left_error * Kp + left_sum * Ki + (left_error - left_last) * Kd;
        const double right_correction = right_error * Kp + right_sum * Ki + (right_error - right_last) * Kd;

        left_last = left_speed;
        right_last = right_speed;
        left_sum += left_speed;
        right_sum += right_speed;

        left_sp = clamp(left_sp - left_correction, -sp_limit, sp_limit);
        right_sp = clamp(right_sp - right_correction, -sp_limit, sp_limit);

        left_wheel.set_duty_cycle_setpoint(left_sp * direction);
        right_wheel.set_duty_cycle_setpoint(right_sp * direction);

        //Logger::info(distance, speed_target);
    }

    left_wheel.stop();
    right_wheel.stop();

    while (left_wheel.get_speed() != 0cm || right_wheel.get_speed() != 0cm) {}
}

inline void steer_around_left (const Angle auto target_angle)
{
    left_wheel.set_stop_action(TachoMotor::stop_actions::hold);
    left_wheel.stop();

    right_wheel.set_duty_cycle_setpoint(0);
    right_wheel.run_direct();

    right_wheel.set_stop_action(TachoMotor::stop_actions::brake);

    const double dir_end = angle_cast<deg>(target_angle).value;
    const double dir_start = gyro.get_angle().value;
    const int direction = (dir_end - dir_start > 0) ? 1 : -1;

    const double max_speed_target = 400;
    double speed_target = max_speed_target;

    int cycles = 0;
    static const int cycles_threshold = 5;

    double right_sp = -20 * direction, right_sum = 0, right_last = 0;

    const double Kp = 0.008, Ki = 0.00000007, Kd = 0.00002;

    while (cycles < cycles_threshold) {
        const double distance = (dir_end - gyro.get_angle().value) * direction;

        if (abs(distance) <= 1) {
            cycles++;
        }
        else {
            cycles = 0;
        }

        if (distance < 60) {
            speed_target = max_speed_target * clamp(0.2 + distance / 60, 0.0, 1.0);
        }

        const double right_speed = right_wheel.get_speed<deg>().value * direction;
        const double right_error = right_speed - (-speed_target);
        const double right_correction = right_error * Kp + right_sum * Ki + (right_error - right_last) * Kd;

        right_last = right_speed;
        right_sum += right_speed;

        right_sp = clamp(right_sp - right_correction, -70.0, 70.0);

        right_wheel.set_duty_cycle_setpoint(right_sp * direction);

        Logger::info(distance, speed_target, right_speed, right_sp);
    }

    right_wheel.stop();

    while (left_wheel.get_speed() != 0cm || right_wheel.get_speed() != 0cm) {}
}

inline void steer_around_right (const Angle auto target_angle)
{
    right_wheel.set_stop_action(TachoMotor::stop_actions::hold);
    right_wheel.stop();

    left_wheel.set_duty_cycle_setpoint(0);
    left_wheel.run_direct();

    left_wheel.set_stop_action(TachoMotor::stop_actions::brake);
    
    const double dir_end = angle_cast<deg>(target_angle).value;
    const double dir_start = gyro.get_angle().value;
    const int direction = (dir_end - dir_start > 0) ? 1 : -1;

    const double max_speed_target = 200;
    double speed_target = max_speed_target;

    int cycles = 0;
    static const int cycles_threshold = 5;

    double left_sp = 20 * direction, left_sum = 0, left_last = 0;

    const double Kp = 0.008, Ki = 0.00000007, Kd = 0.00002;

    while (cycles < cycles_threshold) {
        const double distance = (dir_end - gyro.get_angle().value) * direction;

        if (abs(distance) <= 1) {
            cycles++;
        }
        else {
            cycles = 0;
        }

        if (distance < 60) {
            speed_target = max_speed_target * clamp(0.2 + distance / 60, 0.0, 1.0);
        }

        const double left_speed = left_wheel.get_speed<deg>().value * direction;
        const double left_error = left_speed - speed_target;
        const double left_correction = left_error * Kp + left_sum * Ki + (left_error - left_last) * Kd;

        left_last = left_speed;
        left_sum += left_speed;

        left_sp = clamp(left_sp - left_correction, -70.0, 70.0);

        left_wheel.set_duty_cycle_setpoint(left_sp * direction);

        Logger::info(distance, speed_target, left_speed, left_sp);
    }

    left_wheel.stop();

    while (left_wheel.get_speed() != 0cm || right_wheel.get_speed() != 0cm) {}
}

inline void move_segment (const Unit auto segment, const Angle auto target_angle) 
{
    const int direction = (segment.value > 0) ? 1 : -1;
    SegmentControl control(direction * segment);
    move(direction, target_angle, control);
}

inline void move_wallbang (const Unit auto segment, const Angle auto target_angle)
{
    const int direction = (segment.value > 0) ? 1 : -1;
    SegmentWallbangControl control(direction * segment);
    move(direction, target_angle, control);
}

inline void lift_up ()
{
    static bool first = true;
    if (first) {
        arm.on_for_segment(2 * 360deg, 1050deg);
        first = false;
    }
    else {
        arm.on_for_segment(4 * 360deg, 1050deg);
    }
}

inline void lift_down ()
{
    arm.on_for_segment<false, true>(-4 * 360deg, 300deg);
}

inline void unregulated_move (const int sp, const auto duration)
{
    left_wheel.set_duty_cycle_setpoint(sp);
    right_wheel.set_duty_cycle_setpoint(sp);
    left_wheel.run_direct();
    right_wheel.run_direct();

    left_wheel.set_stop_action(TachoMotor::stop_actions::brake);
    right_wheel.set_stop_action(TachoMotor::stop_actions::brake);

    sleep(duration);

    left_wheel.stop();
    right_wheel.stop();
}

#pragma once

#include <frt/frt.hpp>

#define FRT_ROBOT_ID 0

#if !defined FRT_ROBOT_ID || (FRT_ROBOT_ID != 0) && (FRT_ROBOT_ID != 1)
#error FRT_ROBOT_ID not defined.
#endif

// 5.5 cm wheels with a gear ratio of 3
FRT::TachoMotor left_wheel {OUTPUT_C, FRT::cm(5.5 * 3.0)};
FRT::TachoMotor right_wheel {OUTPUT_B, FRT::cm(5.5 * 3.0)};
// todo: gyro at port 1, sensor needs implementation

#if FRT_ROBOT_ID == 0
// gear has 12 teeth, rack has one tooth per 3.2mm
FRT::TachoMotor linear_motion {OUTPUT_A, FRT::mm(12.0 * 3.2 / M_PI)};
#elif
FRT::TachoMotor brush (OUTPUT_A, 0);
#endif


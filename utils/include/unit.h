#ifndef UNIT_H
#define UNIT_H

static constexpr float PI  = 3.14159265359;
static constexpr float TWO_PI = 6.28318530718;

static constexpr float GEAR_RATIO = 9.0; // 9:1
// 9 revolution of the motor gives 1 revolution of the output shaft there 2 PI rad

static constexpr float MOTOR_TURNS_TO_OUTPUT_TURNS_RAD =  TWO_PI / GEAR_RATIO;
static constexpr float OUTPUT_TURNS_RAD_TO_MOTOR_TURNS = GEAR_RATIO / TWO_PI;



#endif // UNIT_H
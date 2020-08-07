#include "control.h"
#include "mbed.h"

void gesture_control::Angle_PID(double expect_roll,double expect_pitch,double actual_roll,double actual_pitch,double actual_roll_speed,double actual_pitch_speed){

    // 获得误差值
    double e_roll=expect_roll-actual_roll;
    double e_pitch=expect_pitch=actual_pitch;

    // 获得误差积分
    this->i_e_roll+=e_roll*ANGLE_KI;
    this->i_e_pitch+=e_pitch*ANGLE_KI;

    double d_e_roll=e_roll-this->last_e_roll;
    double d_e_pitch=e_pitch-this->last_e_pitch;

    this->output_roll=ANGLE_KP*e_roll+this->i_e_roll+d_e_roll*ANGLE_KD;
    this->output_pitch=ANGLE_KP*e_pitch+this->i_e_pitch+d_e_pitch*ANGLE_KD;

    this->last_e_roll=e_roll;
    this->last_e_pitch=e_pitch;
}

double gesture_control::Get_OutPut_Roll(){
    return this->output_roll;
}

double gesture_control::Get_OutPut_Pitch(){
    return this->output_pitch;
}
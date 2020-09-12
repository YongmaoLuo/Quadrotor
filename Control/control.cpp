#include "control.h"
#include "mbed.h"

void gesture_control::Angle_PID(double expect_roll,double expect_pitch,double actual_roll,double actual_pitch, double actual_yaw, double actual_roll_speed,double actual_pitch_speed){
    /*
    // 默认无头模式
    double head_hold=0;
    
    // 计算偏航角的偏差角度，然后用来得到实际的目标roll和pitch
    double rad_yaw_diff = (actual_yaw - headHold) * PI / 180.0f;
    double cos_yaw_diff = cosf(rad_yaw_diff);
    double sin_yaw_diff = sinf(rad_yaw_diff);
    double target_pitch_free = expect_pitch * cos_yaw_diff + expect_roll * sin_yaw_diff;
    
    expect_roll=expect_roll * cos_yaw_diff - expect_pitch * sin_yaw_diff;
    expect_pitch=target_pitch_free;
    */

    // 获得误差值
    double e_roll=expect_roll-actual_roll;
    double e_pitch=expect_pitch-actual_pitch;

    // 获得误差积分
    this->i_e_roll+=e_roll*ANGLE_KI;
    this->i_e_pitch+=e_pitch*ANGLE_KI;

    double d_e_roll=-actual_roll_speed/ANGLE_PID_FREQUENCY*1.0f;
    double d_e_pitch=-actual_pitch_speed/ANGLE_PID_FREQUENCY*1.0f;

    double satu_output_roll=ANGLE_KP*e_roll+d_e_roll*ANGLE_KD;
    double satu_output_pitch=ANGLE_KP*e_pitch+d_e_pitch*ANGLE_KD;

    if(satu_output_roll>150){
        satu_output_roll=150;
    }else if(satu_output_roll<-150){
        satu_output_roll=-150;
    }
    
    if(satu_output_pitch>150){
        satu_output_pitch=150;
    }else if(satu_output_pitch<-150){
        satu_output_pitch=-150;
    }
    this->output_roll=satu_output_roll;
    this->output_pitch=satu_output_pitch;

    this->last_e_roll=e_roll;
    this->last_e_pitch=e_pitch;
}

double gesture_control::Get_OutPut_Roll(){
    return this->output_roll;
}

double gesture_control::Get_OutPut_Pitch(){
    return this->output_pitch;
}

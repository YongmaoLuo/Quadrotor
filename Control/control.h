#ifndef PANDAFLYCONTROL
#define PANDAFLYCONTROL

#include "mbed.h"
#include "../InOut/command.h"
#include "../InOut/imu.h"

#define ANGLE_KP 2.5f
#define ANGLE_KI 0.0005f
#define ANGLE_KD 20.0f
#define ANGLE_PID_FREQUENCY 200.0f // 200Hz的PID控制频率
#define ANGLE_VELOCITY_KP
#define ANGLE_VELOCITY_KI
#define ANGLE_VELOCITY_KD

class location_control {
};

class gesture_control {
    double last_e_roll=0,last_e_pitch=0;//储存偏差值
    double i_e_roll=0,i_e_pitch=0;//积分项
    double output_roll=0,output_pitch=0;
public:
    void Angle_PID(double expect_roll,double expect_pitch,double actual_roll,double actual_pitch, double actual_yaw,double actual_roll_speed,double actual_pitch_speed);
    void Angle_Velocity_PID();
    double Get_OutPut_Roll();
    double Get_OutPut_Pitch();
};


#endif

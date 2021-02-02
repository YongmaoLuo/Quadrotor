#ifndef PANDAFLYCONTROL
#define PANDAFLYCONTROL

#include "mbed.h"
#include "../InOut/command.h"
#include "../InOut/imu.h"

#define ANGLE_KP 2.5f
#define ANGLE_VELOCITY_KP 2.0f
#define ANGLE_VELOCITY_KI 0.05f
#define ANGLE_VELOCITY_KD 1.0f
#define PID_FREQUENCY 200.0f // 200Hz的PID控制频率

class location_control {
};

class gesture_control {
    static float output_roll,output_pitch;
    static float i_roll_speed,i_pitch_speed;
    static float last_roll_speed,last_pitch_speed;
    static float output_roll_speed,output_pitch_speed;
    static Mutex roll_speed_lock,pitch_speed_lock;
    
public:
    gesture_control();
    static void Angle_PID(float expect_roll,float expect_pitch,float actual_roll,float actual_pitch);
    static void Angle_Velocity_PID(float actual_roll_speed,float actual_pitch_speed);
    float Get_OutPut_Roll_Speed();
    float Get_OutPut_Pitch_Speed();
};


#endif

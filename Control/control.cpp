#include "control.h"
#include "mbed.h"

gesture_control::gesture_control(){
    output_roll=0,output_pitch=0;
    i_roll_speed=0,i_pitch_speed=0;
    last_roll_speed=0,last_pitch_speed=0;
}

void gesture_control::Angle_PID(float expect_roll,float expect_pitch,float actual_roll,float actual_pitch){
    
    // 获得误差值
    float e_roll=expect_roll-actual_roll;
    float e_pitch=expect_pitch-actual_pitch;

    float satu_output_roll=ANGLE_KP*e_roll;
    float satu_output_pitch=ANGLE_KP*e_pitch;

    output_roll=satu_output_roll;
    output_pitch=satu_output_pitch;
}

void gesture_control::Angle_Velocity_PID(float actual_roll_speed,float actual_pitch_speed){
    float e_roll_speed=output_roll-actual_roll_speed;
    float e_pitch_speed=output_pitch-actual_pitch_speed;
    float d_roll_speed,d_pitch_speed;

    i_roll_speed+=e_roll_speed/PID_FREQUENCY;
    i_pitch_speed+=e_pitch_speed/PID_FREQUENCY;

    if(last_roll_speed==0){
        d_roll_speed=0;
    }else{
        d_roll_speed=(actual_roll_speed-last_roll_speed)*PID_FREQUENCY;
    }
    if(last_pitch_speed==0){
        d_pitch_speed=0;
    }else{
        d_pitch_speed=(actual_pitch_speed-last_pitch_speed)*PID_FREQUENCY;
    }

    double temp_roll_speed=ANGLE_VELOCITY_KP*e_roll_speed+i_roll_speed*ANGLE_VELOCITY_KI+d_roll_speed*ANGLE_VELOCITY_KD;
    double temp_pitch_speed=ANGLE_VELOCITY_KP*e_pitch_speed+i_pitch_speed*ANGLE_VELOCITY_KI+d_pitch_speed*ANGLE_VELOCITY_KD;

    if(temp_roll_speed>100){
        temp_roll_speed=100;
    }else if(temp_roll_speed<-150){
        temp_roll_speed=-150;
    }
    
    if(temp_pitch_speed>150){
        temp_pitch_speed=150;
    }else if(temp_pitch_speed<-150){
        temp_pitch_speed=-150;
    }

    roll_speed_lock.lock();pitch_speed_lock.lock();
    output_roll_speed=temp_roll_speed;
    output_pitch_speed=temp_pitch_speed;
    roll_speed_lock.unlock();pitch_speed_lock.unlock();

    last_roll_speed=actual_roll_speed;
    last_pitch_speed=actual_pitch_speed;

}

float gesture_control::Get_OutPut_Roll_Speed(){
    double result;
    roll_speed_lock.lock();
    result=output_roll_speed;
    roll_speed_lock.unlock();
    return result;
}

float gesture_control::Get_OutPut_Pitch_Speed(){
    double result;
    pitch_speed_lock.lock();
    result=output_pitch_speed;
    pitch_speed_lock.unlock();
    return result;
}

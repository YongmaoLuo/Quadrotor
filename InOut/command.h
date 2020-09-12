#ifndef PANDAFLYCOMMAND
#define PANDAFLYCOMMAND

#define PPM_CHANNELS 8

#include "mbed.h"
#include "../Control/control.h"

class ppm_pwm_command {
    
    PwmOut *motor1, *motor2, *motor3, *motor4;
    
    static Timer timer_ppm;
    static int current_channel;
    static int64_t channels[PPM_CHANNELS + 1];
    static bool state;
    static Mutex ch_lock[PPM_CHANNELS];

public:
    
    InterruptIn *ppm_in;

    ppm_pwm_command(PinName in1, PinName out1, PinName out2, PinName out3,
                    PinName out4);
    void Receive_PPM();
    double Get_Roll_Command();
    double Get_Pitch_Command();
    double Get_Yaw();
    int Get_Throttle();
    bool Get_Fly_Allowance();
    void Output_To_Motor(int throttle,double pitch,double roll,double yaw);

    static void Store_Channel();
    
};

#endif
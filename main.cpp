#include "./InOut/command.h"
#include "./InOut/imu.h"
#include "./Control/control.h"
#include "mbed.h"

#define NEED_CHANNELS 4

BufferedSerial pc(USBTX, USBRX);
FileHandle *mbed::mbed_override_console(int fd) { return &pc; }

ppm_pwm_command thisCommand(p14, p21, p22, p23, p24);
gesture_control thisGesture;
imu thisIMU(p9, p10);

void Control_Thread() {
    thisGesture.Angle_PID(thisCommand.Get_Roll_Command(),thisCommand.Get_Pitch_Command(),thisIMU.Get_Roll(),thisIMU.Get_Pitch(),thisIMU.Get_Roll_Speed(),thisIMU.Get_Pitch_Speed());
    thisCommand.Output_To_Motor(thisCommand.Get_Throttle(),thisCommand.Get_Pitch_Command(),thisCommand.Get_Roll_Command(),thisCommand.Get_Yaw());
}

void IMU_Thread() {
    // 不断读取IMU数据
    // 不断接收指令
    // 不断利用PID算法计算控制量
    thisIMU.ADXL345_ReadData();
    thisIMU.ITG3205_ReadData();
    thisIMU.Mahony_Filter_Update();
}

void receive_ppm_thread() { 
    thisCommand.Receive_PPM(); 
}
// main() runs in its own thread in the OS
int main() {
    pc.set_baud(9600);// 设置调试时与计算机串口通信的波特率
    // IMU初始化与校准
    int ok=0;
    ok=thisIMU.ADXL345_Initialize();
    ok=thisIMU.ADXL345_Calibration();
    ok=thisIMU.ITG3205_Initialize();
    ok=thisIMU.ITG3205_Calibration();
    if(ok!=0){
        while(true){
            ok=thisIMU.ADXL345_Initialize();
            ok=thisIMU.ITG3205_Initialize();
        }
    }
    thisIMU.Mahony_Filter_Init();

    // 创建一个子线程，专门用来接收PPM信号
    Thread receive_ppm;
    receive_ppm.start(receive_ppm_thread);

    // 创建两个计时器，定时执行读取IMU和PID计算的工作
    Timer imu_ticker_100Hz, control_ticker_50Hz;
    control_ticker_50Hz.start();
    imu_ticker_100Hz.start();
    while(true){
        if(control_ticker_50Hz.elapsed_time().count()>=20000){
            control_ticker_50Hz.reset();
            Control_Thread();
        }
        if(imu_ticker_100Hz.elapsed_time().count()>=10000){
            imu_ticker_100Hz.reset();
            IMU_Thread();
        }
    }
    receive_ppm.join();
}

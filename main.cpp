#include "./Control/control.h"
#include "./InOut/command.h"
#include "./InOut/imu.h"
#include "mbed.h"

#define NEED_CHANNELS 5
#define MAHONY_INITIAL_READ_TIMES 100

BufferedSerial pc(USBTX, USBRX);
DigitalOut normal_state(LED4),wrong(LED2);
FileHandle *mbed::mbed_override_console(int fd) { return &pc; }

ppm_pwm_command thisCommand(p14, p21, p22, p23, p24);
gesture_control thisGesture;
imu thisIMU(p9, p10);

int Thread_200Hz() {
    int ok=thisIMU.ADXL345_ReadData();
    thisGesture.Angle_PID(thisCommand.Get_Roll_Command(),
                          thisCommand.Get_Pitch_Command(), thisIMU.Get_Roll(),
                          thisIMU.Get_Pitch(), thisIMU.Get_Roll_Speed(),
                          thisIMU.Get_Pitch_Speed());
    if(thisCommand.Get_Fly_Allowance()){
        thisCommand.Output_To_Motor(
        thisCommand.Get_Throttle(), thisGesture.Get_OutPut_Pitch(),
        thisGesture.Get_OutPut_Roll(), thisCommand.Get_Yaw());
    }else{
        thisCommand.Output_To_Motor(950, 0,0, 0);
    }
    return ok;
    //printf("%d,%f,%f,%f\n",thisCommand.Get_Throttle(),thisGesture.Get_OutPut_Pitch(),thisGesture.Get_OutPut_Roll(),thisCommand.Get_Yaw());
    
}

void Thread_100Hz() {
    // 不断读取加速度计的数据
    
    //return ok;
}

int Thread_500Hz(){
    //不断读取陀螺仪数据
    int ok=thisIMU.ITG3205_ReadData();
    // 更新姿态
    //thisIMU.Mahony_Filter_Update();
    
    thisIMU.Traditional_Linear_Filter_Update();
    printf("roll: %f, pitch: %f, yaw: %f\n", thisIMU.Get_Roll(),thisIMU.Get_Pitch(), thisIMU.Get_Yaw());
    return ok;
}

void receive_ppm_thread() { thisCommand.Receive_PPM(); }
// main() runs in its own thread in the OS
int main() {
    wrong=0;
    normal_state=0;
    pc.set_baud(9600); // 设置调试时与计算机串口通信的波特率
    // IMU初始化与校准
    int ok = 0;
    ok = thisIMU.ADXL345_Initialize();
    ok=thisIMU.ADXL345_Calibration();
    ok = thisIMU.ITG3205_Initialize();
    ok = thisIMU.ITG3205_Calibration();
    if (ok != 0) {
        while (true) {
            ok = thisIMU.ADXL345_Initialize();
            ok = thisIMU.ITG3205_Initialize();
            wrong=1;
        }
    }
    Timer temp_timer;
    int adxl_read_times=0;
    double average_acc_x=0,average_acc_y=0,average_acc_z=-1;
    temp_timer.start();
    
    while(1){
        if(temp_timer.elapsed_time().count()>=10000){
            temp_timer.reset();
            adxl_read_times++;
            thisIMU.ADXL345_ReadData();
            average_acc_x+=thisIMU.Get_ACC_X();
            average_acc_y+=thisIMU.Get_ACC_Y();
            average_acc_z+=thisIMU.Get_ACC_Z();
        }
        if(adxl_read_times>=MAHONY_INITIAL_READ_TIMES){
            temp_timer.stop();
            break;
        }
    }
    average_acc_x/=MAHONY_INITIAL_READ_TIMES;
    average_acc_y/=MAHONY_INITIAL_READ_TIMES;
    average_acc_z/=MAHONY_INITIAL_READ_TIMES;
    //thisIMU.Mahony_Filter_Init(average_acc_x,average_acc_y,average_acc_z);
    thisIMU.Traditional_Linear_Filter_Init(average_acc_x,average_acc_y,average_acc_z);

    // 创建一个子线程，专门用来接收PPM信号
    Thread receive_ppm;
    receive_ppm.start(receive_ppm_thread);

    // 创建两个计时器，定时执行读取IMU和PID计算的工作
    Timer ticker_100Hz, ticker_200Hz,ticker_500Hz;
    ticker_200Hz.start();
    ticker_100Hz.start();
    ticker_500Hz.start();
    normal_state=1;
    while (true) {
        if(ticker_500Hz.elapsed_time().count()>=2000){
            ticker_500Hz.reset();
            ok=Thread_500Hz();
        }
        if (ticker_200Hz.elapsed_time().count() >= 5000) {
            ticker_200Hz.reset();
            ok=Thread_200Hz();
        }
        if (ticker_100Hz.elapsed_time().count() >= 10000) {
            ticker_100Hz.reset();
            //ok=Thread_100Hz();
        }
        if(ok!=0){
            wrong=1;
            normal_state=0;
        }
    }
    receive_ppm.join();
}

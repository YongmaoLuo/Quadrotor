#include "./Control/control.h"
#include "./InOut/command.h"
#include "./InOut/imu.h" 
#include "mbed.h"
#include <map>

#define NEED_CHANNELS 5
#define MAHONY_INITIAL_READ_TIMES 100

#define THREAD_MAHONY 0
#define THREAD_ACC 1
#define THREAD_GYR 2
#define THREAD_PID 3
#define THREAD_MOTOR 4
#define THREAD_NUMBER 5

//system's classes
BufferedSerial pc(USBTX, USBRX);
DigitalOut normal_state(LED4),wrong_led(LED2);
FileHandle *mbed::mbed_override_console(int fd) { return &pc; }

// own classes
ppm_pwm_command thisCommand(p14, p21, p22, p23, p24);
gesture_control thisGesture;
imu thisIMU(p9, p10);

// instantiate the static members of class "ppm_pwm_command".
Timer ppm_pwm_command::timer_ppm;
int ppm_pwm_command::current_channel;
int64_t ppm_pwm_command::channels[PPM_CHANNELS + 1];
bool ppm_pwm_command::state;
Mutex ppm_pwm_command::ch_lock[PPM_CHANNELS];

// Scheduling part
struct OSTCB{
    int os_tcb_delay;
    Thread os_task_thread;
};
OSTCB task_receive_ppm,task_mahony,task_acc,task_gyr,task_pid,task_motor;
EventQueue queue;
int thread_delayed_time[THREAD_NUMBER];
osPriority_t os_priority_now;
//std::map<osPriority_t,>
Thread thread_receive_ppm(osPriorityNormal7),thread_mahony(osPriorityNormal6),
    thread_acc(osPriorityNormal5),thread_gyr(osPriorityNormal5),
    thread_pid(osPriorityNormal4),thread_motor(osPriorityNormal4);

void OS_Time_Tick(){
    for(int i=0;i<THREAD_NUMBER;i++){
        thread_delayed_time[i]--;
        if(thread_delayed_time[i]==0){
            switch(i){
                case 0:
                    if(os_priority_now<osPriorityNormal7){

                    }
                    thread_delayed_time[i]=4;
                    break;
                case 1: 
                    thread_delayed_time[i] = 5;
                    break;
                case 2: 
                    thread_delayed_time[i]= 4;
                    break;
                case 3: 
                    thread_delayed_time[i]= 20;
                    break;
                case 4: 
                    thread_delayed_time[i]= 20;
                    break;
            }
        }
    }
}

int Thread_200Hz() {
    
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
    printf("%d,%f,%f,%f\n",thisCommand.Get_Throttle(),thisGesture.Get_OutPut_Pitch(),thisGesture.Get_OutPut_Roll(),thisCommand.Get_Yaw());

    //return ok;
    
}

int Thread_800Hz() {
    // 不断读取加速度计的数据
    int ok=thisIMU.ADXL345_ReadData();
    return ok;
}

int Thread_1000Hz(){
    // read data from gyroscope
    int ok=thisIMU.ITG3205_ReadData();
    return ok;
}

void Thread_5000Hz(){
    // update attitude
    thisIMU.Mahony_Filter_Update();
    //thisIMU.Traditional_Linear_Filter_Update();
    printf("roll: %f, pitch: %f, yaw: %f\n", thisIMU.Get_Roll(),thisIMU.Get_Pitch(), thisIMU.Get_Yaw());
    return;
}

void receive_ppm_thread() { 
    ppm_pwm_command::Store_Channel();
}

void OS_Init(){
    wrong_led=0;
    normal_state=0;
    pc.set_baud(9600);

    // IMU initialization and calibration
    int ok = 0;
    ok = thisIMU.ADXL345_Initialize();
    ok=thisIMU.ADXL345_Calibration();
    ok = thisIMU.ITG3205_Initialize();
    ok = thisIMU.ITG3205_Calibration();
    if (ok != 0) {
        while (true) {
            ok = thisIMU.ADXL345_Initialize();
            ok = thisIMU.ITG3205_Initialize();
            wrong_led=1;
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

    thisIMU.Mahony_Filter_Init(average_acc_x,average_acc_y,average_acc_z);
}

// main() runs in its own thread in the OS
int main() {
    OS_Init();
    
    // set time delay for each task
    thread_delayed_time[THREAD_MAHONY]=4;
    thread_delayed_time[THREAD_ACC]=5;
    thread_delayed_time[THREAD_GYR]=4;
    thread_delayed_time[THREAD_PID]=20;
    thread_delayed_time[THREAD_MOTOR]=20;

    Ticker ticker_250us;
    ticker_250us.attach(&OS_Time_Tick, 250us);
    
    normal_state=1;
    while (true) {
        
    }

}

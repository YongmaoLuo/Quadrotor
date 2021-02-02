#include "./Control/control.h"
#include "./InOut/command.h"
#include "./InOut/imu.h"
#include "mbed.h"
#include <map>
//#include <atomic>

#define MAHONY_INITIAL_READ_TIMES 100

void OS_Init();
void IMU_Init();
void Thread_Init();
void Task_Receive_PPM();
void Task_Mahony();
void Task_ACC();
void Task_GYR();
void Task_Series_PID();
void Task_Motor();

// system's classes
BufferedSerial pc(USBTX, USBRX);
DigitalOut normal_state(LED4), wrong_led(LED2);
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
PwmOut *ppm_pwm_command::motor1, *ppm_pwm_command::motor2,
    *ppm_pwm_command::motor3, *ppm_pwm_command::motor4;

// instantiate the static members of class "imu"
I2C *imu::i2c_imu;
char imu::acc_data[7], imu::gyr_data[7]; //加速度计三轴加速度
Mutex imu::gyr_lock;
Mutex imu::acc_lock;
Mutex imu::roll_lock, imu::pitch_lock, imu::yaw_lock;
float imu::q0, imu::q1, imu::q2, imu::q3; //表示整个旋转的四元数
float imu::exInt, imu::eyInt, imu::ezInt; //加速度计和陀螺仪之间的测量误差
float imu::roll, imu::pitch, imu::yaw, imu::last_roll, imu::last_pitch,
    imu::last_yaw;
float imu::gyr_offsetx, imu::gyr_offsety,
    imu::gyr_offsetz; //陀螺仪三轴角速度，和三轴初始误差

// instantiate the static members of class "gesture_control"
float gesture_control::output_roll, gesture_control::output_pitch;
float gesture_control::i_roll_speed, gesture_control::i_pitch_speed;
float gesture_control::last_roll_speed, gesture_control::last_pitch_speed;
float gesture_control::output_roll_speed, gesture_control::output_pitch_speed;
Mutex gesture_control::roll_speed_lock, gesture_control::pitch_speed_lock;

// Scheduling part
std::map<osPriority_t, Thread *> thread_map;
std::map<osPriority_t, EventQueue *> queue_map;

void IRQ_Receive_PPM() {
    queue_map[osPriorityBelowNormal7]->call(&Task_Receive_PPM);
}

void IRQ_ACC() { queue_map[osPriorityBelowNormal7]->call(&Task_ACC); }

void IRQ_GYR() { queue_map[osPriorityBelowNormal7]->call(&Task_GYR); }

void IRQ_Motor() { queue_map[osPriorityBelowNormal6]->call(&Task_Motor); }

void IRQ_Mahony() { queue_map[osPriorityBelowNormal5]->call(&Task_Mahony); }

void IRQ_Series_PID() {
    queue_map[osPriorityBelowNormal5]->call(&Task_Series_PID);
}

void Task_Receive_PPM() {
    thisCommand.Store_Channel();
    queue_map[osPriorityBelowNormal4]->call(printf, "%f\n",
                                            thisCommand.Get_Throttle());
    // queue_map[osPriorityBelowNormal4]->call(printf,"task receive ppm is
    // running!\n");
}

void Task_Mahony() {
    thisIMU.Mahony_Filter_Update();
    // queue_map[osPriorityBelowNormal4]->call(printf,"task mahony is
    // running!\n"); queue_map[osPriorityBelowNormal4]->call(printf,"%f, %f,
    // %f\n",thisIMU.Get_Roll(),thisIMU.Get_Pitch(),thisIMU.Get_Yaw());
}

void Task_ACC() {
    thisIMU.ADXL345_ReadData();
    // queue_map[osPriorityBelowNormal4]->call(printf,"task acc is running!\n");
}

void Task_GYR() { thisIMU.ITG3205_ReadData(); }

void Task_Series_PID() {
    thisGesture.Angle_PID(thisCommand.Get_Roll_Command(),
                          thisCommand.Get_Pitch_Command(), thisIMU.Get_Roll(),
                          thisIMU.Get_Pitch());
    thisGesture.Angle_Velocity_PID(thisIMU.Get_Roll_Speed(),
                                   thisIMU.Get_Pitch_Speed());
}

void Task_Motor() {
    if (thisCommand.Get_Fly_Allowance()) {
        thisCommand.Output_To_Motor(thisGesture.Get_OutPut_Pitch_Speed(),
                                    thisGesture.Get_OutPut_Roll_Speed());
    } else {
        thisCommand.Output_To_Motor(0, 0);
    }
    // printf("%d,%f,%f,%f\n",thisCommand.Get_Throttle(),thisGesture.Get_OutPut_Pitch_Speed(),thisGesture.Get_OutPut_Roll_Speed(),thisCommand.Get_Yaw());
}

void IRQ_Task_Pool_Start_PN7() {
    queue_map[osPriorityBelowNormal7]->dispatch_forever();
}

void IRQ_Task_Pool_Start_PN6() {
    queue_map[osPriorityBelowNormal6]->dispatch_forever();
}

void IRQ_Task_Pool_Start_PN5() {
    queue_map[osPriorityBelowNormal5]->dispatch_forever();
}

void IRQ_Task_Pool_Start_PN4() {
    queue_map[osPriorityBelowNormal4]->dispatch_forever();
}

// main() runs in its own thread in the OS
int main() {
    OS_Init();
    printf("os init completed\n");

    // threads to schedule tasks
    thread_map[osPriorityBelowNormal7]->start(callback(
        queue_map[osPriorityBelowNormal7], &EventQueue::dispatch_forever));
    thread_map[osPriorityBelowNormal6]->start(callback(
        queue_map[osPriorityBelowNormal6], &EventQueue::dispatch_forever));
    thread_map[osPriorityBelowNormal5]->start(callback(
        queue_map[osPriorityBelowNormal5], &EventQueue::dispatch_forever));
    thread_map[osPriorityBelowNormal4]->start(callback(
        queue_map[osPriorityBelowNormal4], &EventQueue::dispatch_forever));
    printf("thread started\n");

    thisCommand.Initializing_Receiving_PPM();
    printf("init receiving ppm success\n");
    thisCommand.ppm_in->rise(
        queue_map[osPriorityBelowNormal7]->event(Task_Receive_PPM));
    printf("begin receiving\n");

    // interrupt periodically to create tasks
    Ticker ticker_mahony, ticker_acc, ticker_gyr, ticker_pid, ticker_motor;

    ticker_mahony.attach(queue_map[osPriorityBelowNormal5]->event(Task_Mahony),
                         2000us);
    ticker_pid.attach(queue_map[osPriorityBelowNormal5]->event(Task_Series_PID),
                      5000us);
    ticker_motor.attach(queue_map[osPriorityBelowNormal6]->event(Task_Motor),
                        5000us);
    ticker_acc.attach(queue_map[osPriorityBelowNormal7]->event(Task_ACC),
                      1250us);
    ticker_gyr.attach(queue_map[osPriorityBelowNormal7]->event(Task_GYR),
                      1000us);

    normal_state = 1;

    // while(1){}

    thread_map[osPriorityBelowNormal7]->join();
    thread_map[osPriorityBelowNormal6]->join();
    thread_map[osPriorityBelowNormal5]->join();
    thread_map[osPriorityBelowNormal4]->join();
}

void IMU_Init() {
    // IMU initialization and calibration
    int ok = 0;
    ok = thisIMU.ADXL345_Initialize();
    ok = thisIMU.ADXL345_Calibration();
    ok = thisIMU.ITG3205_Initialize();
    ok = thisIMU.ITG3205_Calibration();
    if (ok != 0) {
        while (true) {
            ok = thisIMU.ADXL345_Initialize();
            ok = thisIMU.ITG3205_Initialize();
            wrong_led = 1;
        }
    }
    Timer temp_timer;
    int adxl_read_times = 0;
    double average_acc_x = 0, average_acc_y = 0, average_acc_z = -1;
    temp_timer.start();
    while (1) {
        if (temp_timer.elapsed_time().count() >= 10000) {
            temp_timer.reset();
            adxl_read_times++;
            thisIMU.ADXL345_ReadData();
            average_acc_x += thisIMU.Get_ACC_X();
            average_acc_y += thisIMU.Get_ACC_Y();
            average_acc_z += thisIMU.Get_ACC_Z();
        }
        if (adxl_read_times >= MAHONY_INITIAL_READ_TIMES) {
            temp_timer.stop();
            break;
        }
    }
    average_acc_x /= MAHONY_INITIAL_READ_TIMES;
    average_acc_y /= MAHONY_INITIAL_READ_TIMES;
    average_acc_z /= MAHONY_INITIAL_READ_TIMES;

    thisIMU.Mahony_Filter_Init(average_acc_x, average_acc_y, average_acc_z);
}

void Thread_Init() {
    // set event queue
    queue_map[osPriorityBelowNormal7] = new EventQueue(EVENTS_QUEUE_SIZE);
    queue_map[osPriorityBelowNormal6] = new EventQueue(EVENTS_QUEUE_SIZE);
    queue_map[osPriorityBelowNormal5] = new EventQueue(EVENTS_QUEUE_SIZE);
    queue_map[osPriorityBelowNormal4] =
        new EventQueue(15 * 4 * 10); // printf has two arguments

    thread_map[osPriorityBelowNormal7] =
        new Thread(osPriorityBelowNormal7,
                   EVENTS_QUEUE_SIZE + sizeof(mbed::Callback<void()>));
    thread_map[osPriorityBelowNormal6] =
        new Thread(osPriorityBelowNormal6,
                   EVENTS_QUEUE_SIZE + sizeof(mbed::Callback<void()>));
    thread_map[osPriorityBelowNormal5] =
        new Thread(osPriorityBelowNormal5,
                   EVENTS_QUEUE_SIZE + sizeof(mbed::Callback<void()>));
    thread_map[osPriorityBelowNormal4] =
        new Thread(osPriorityBelowNormal4,
                   EVENTS_QUEUE_SIZE + sizeof(mbed::Callback<void()>));
}

void OS_Init() {

    pc.set_baud(9600);
    IMU_Init();
    Thread_Init();
}
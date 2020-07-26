#include "mbed.h"
#include "IMU/imu.h"

BufferedSerial pc(USBTX,USBRX);

FileHandle *mbed::mbed_override_console(int fd)
{
    return &pc;
}


// main() runs in its own thread in the OS
int main() {
    int ok=0;
    imu thisIMU(p9,p10);
    ok=thisIMU.ADXL345_Initialize();
    ok=thisIMU.ADXL345_Calibration();
    ok=thisIMU.ITG3205_Initialize();
    thisIMU.ITG3205_Calibration();
    if(ok!=0){
        while(true){
            ok=thisIMU.ADXL345_Initialize();
            ok=thisIMU.ITG3205_Initialize();
        }
    }
    pc.set_baud(9600);
    // 系统初始化
    while (true) {
        // 不断读取IMU数据
        // 不断接收指令
        // 不断利用PID算法计算控制量
        thisIMU.ADXL345_ReadData();
        thisIMU.ITG3205_ReadData();
        double roll = atan2(thisIMU.acc_x, thisIMU.acc_y) * 180/PI;
        double pitch = atan2(-thisIMU.acc_x, sqrt(thisIMU.acc_y*thisIMU.acc_y + thisIMU.acc_z*thisIMU.acc_z)) * 180/PI;
        //printf("x:%f, y:%f, z:%f\n",thisIMU.acc_x,thisIMU.acc_y,thisIMU.acc_z);
        //printf("x:%f, y:%f, z:%f\n",thisIMU.gyr_x,thisIMU.gyr_y,thisIMU.gyr_z);
        //printf("Roll: %f, Pitch: %f\n",roll,pitch);
        wait_us(10000);
    }
}

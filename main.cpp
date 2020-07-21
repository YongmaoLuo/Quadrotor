#include "mbed.h"
#include "IMU/imu.h"

BufferedSerial pc(USBTX,USBRX);

FileHandle *mbed::mbed_override_console(int fd)
{
    return &pc;
}


// main() runs in its own thread in the OS
int main() {
    bool ok=true;
    imu thisIMU(p9,p10);
    ok=thisIMU.ITG3205_Initialize();
    if(!ok){
        while(true){
            ok=thisIMU.ITG3205_Initialize();
        }
    }
    pc.set_baud(9600);
    // 系统初始化
    while (true) {
        // 不断读取IMU数据
        // 不断接收指令
        // 不断利用PID算法计算控制量
        thisIMU.ITG3205_ReadData();
        printf("x:%f, y:%f, z:%f\n",thisIMU.acc_x,thisIMU.acc_y,thisIMU.acc_z);
        //wait_ns(10000000);
    }
}

#ifndef PANDAFLYIMU
#define PANDAFLYIMU

#define ALPHA 0.5 //低通滤波参数
#define PI 3.14

#define ADXL345_ADDRESS_8BITS 0x53<<1
#define ADXL345_RATIO 0.003906 // ADXL345精确度
#define ADXL345_OFFSETX 0x1E
#define ADXL345_OFFSETY 0x1F
#define ADXL345_OFFSETZ 0x20
#define ADXL345_POWER_CTL 0x2D 
#define ADXL345_DATA_FORMAT 0x31 
#define ADXL345_BW_RATE 0x2C 
#define ADXL345_800HZ 0x0D
#define ADXL345_1600HZ 0x0E

#define ITG3205_ADDRESS_8BITS 0x68<<1
#define ITG3205_SENSITIVITY 14.375
#define ITG3205_SAMPLE_RATE_DIVIDER 0x15
#define ITG3205_DLPF_FS 0x16
#define ITG3205_POWER_MANAGEMENT 0x3E
#define ITG3205_XH 0x1D
#define ITG3205_TEMPH 0x1B

#define HMC5883L_ADDRESS_8BITS 0x3C<<1

#define GYR_KP 1.0f //kp，ki用于控制加速度计修正陀螺仪积分姿态的速度
#define GYR_KI 0.05f
#define DT 0.01 // 两次解算之间的时间间隔，单位为s

#include "mbed.h"

class imu{
    I2C *i2c_imu=nullptr;
    double acc_x=0,acc_y=0,acc_z=0;//加速度计三轴加速度
    double gyr_x,gyr_y,gyr_z,gyr_offsetx=0,gyr_offsety=0,gyr_offsetz=0;//陀螺仪三轴角速度，和三轴初始误差
    double q0,q1,q2,q3;//表示整个旋转的四元数
    double exInt=0,eyInt=0,ezInt=0;//加速度计和陀螺仪之间的测量误差
    double roll,pitch,yaw;

public:
    imu(PinName sda,PinName scl);

    int ADXL345_Initialize();
    int ADXL345_ReadData();
    int ADXL345_Calibration();
    int ITG3205_Initialize();
    int ITG3205_ReadData();
    int ITG3205_Calibration();
    int HMC5883L_Initialize();
    int HMC5883L_ReadData();

    void Mahony_Filter_Init();
    void Mahony_Filter_Update();
    double Get_Roll();
    double Get_Pitch();
    double Get_Yaw();
    double Get_Roll_Speed();
    double Get_Pitch_Speed();
    double Get_Yaw_Speed();

};

#endif
#ifndef MBEDFLYIMU
#define MBEDFLYIMU

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

#include "mbed.h"
class imu{
    I2C *i2c_imu=nullptr;

public:
    double acc_x=0,acc_y=0,acc_z=0;
    double gyr_x,gyr_y,gyr_z,gyr_offsetx=0,gyr_offsety=0,gyr_offsetz=0;
    imu(PinName sda,PinName scl);
    int ADXL345_Initialize();
    int ADXL345_ReadData();
    int ADXL345_Calibration();
    int ITG3205_Initialize();
    int ITG3205_ReadData();
    int ITG3205_Calibration();
    int HMC5883L_Initialize();
    int HMC5883L_ReadData();
};

#endif
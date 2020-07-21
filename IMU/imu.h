#ifndef MBEDFLYIMU
#define MBEDFLYIMU

#define ALPHA 0.5 //低通滤波参数

#define ADXL345_ADDRESS_8BITS 0x53<<1
#define ADXL345_RATIO 0.0039 // ADXL345精确度
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
    double acc_x=0,acc_y=0,acc_z=0,gyr_x,gyr_y,gyr_z;
    imu(PinName sda,PinName scl);
    bool ADXL345_Initialize();
    void ADXL345_ReadData();
    bool ITG3205_Initialize();
    void ITG3205_ReadData();
    bool HMC5883L_Initialize();
    void HMC5883L_ReadData();
};

#endif
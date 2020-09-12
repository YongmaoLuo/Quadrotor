#ifndef PANDAFLYIMU
#define PANDAFLYIMU

#define ALPHA 0.4f //低通滤波参数
#define PI 3.1415926f

#define ADXL345_ADDRESS_8BITS 0x53 << 1
#define ADXL345_RATIO 0.003906f // ADXL345精确度
#define ADXL345_OFFSETX 0x1E
#define ADXL345_OFFSETY 0x1F
#define ADXL345_OFFSETZ 0x20
#define ADXL345_POWER_CTL 0x2D
#define ADXL345_DATA_FORMAT 0x31
#define ADXL345_BW_RATE 0x2C
#define ADXL345_800HZ 0x0D
#define ADXL345_1600HZ 0x0E

#define ITG3205_ADDRESS_8BITS 0x68 << 1
#define ITG3205_SENSITIVITY 14.375f
#define ITG3205_SAMPLE_RATE_DIVIDER 0x15
#define ITG3205_DLPF_FS 0x16
#define ITG3205_POWER_MANAGEMENT 0x3E
#define ITG3205_XH 0x1D
#define ITG3205_TEMPH 0x1B

#define HMC5883L_ADDRESS_8BITS 0x3C << 1

#define GYR_KP 10.0f // kp，ki用于控制加速度计修正陀螺仪积分姿态的速度
#define GYR_KI 0.0f
#define GYR_KD 0.0f
#define SAMPLE_RATE 5000.0f // 每秒中解算次数
#define TAO 0.0003f //时间常数,单位为s
#define ACC_BEFORE_NUMBER 100 // 储存之前加速度计数据的数组长度
#define GYR_BEFORE_NUMBER 100 // 储存之前陀螺仪数据的数组长度
#define EULER_BEFORE_NUMBER 100

#include "mbed.h"

class imu {
    I2C *i2c_imu = nullptr;
    float acc_x = 0, acc_y = 0, acc_z = 0; //加速度计三轴加速度
    float before_acc_x[ACC_BEFORE_NUMBER]={0},before_acc_y[ACC_BEFORE_NUMBER]={0},before_acc_z[ACC_BEFORE_NUMBER]={0};//储存前十个加速度计的值
    float before_gyr_x[GYR_BEFORE_NUMBER]={0},before_gyr_y[GYR_BEFORE_NUMBER]={0},before_gyr_z[GYR_BEFORE_NUMBER]={0};
    int before_acc_point_x=0,before_acc_point_y=0,before_acc_point_z=0,before_gyr_point_x=0,before_gyr_point_y=0,before_gyr_point_z=0;
    float gyr_x, gyr_y, gyr_z,
        gyr_offsetx = 0, gyr_offsety = 0,
        gyr_offsetz = 0; //陀螺仪三轴角速度，和三轴初始误差
    float q0 = 0, q1 = 0, q2 = 0, q3 = 0; //表示整个旋转的四元数
    float exInt = 0, eyInt = 0, ezInt = 0; //加速度计和陀螺仪之间的测量误差
    float roll, pitch, yaw,last_roll,last_pitch,last_yaw;
    Mutex gyr_x_lock,gyr_y_lock,gyr_z_lock;
    Mutex acc_x_lock,acc_y_lock,acc_z_lock;
    Mutex roll_lock,pitch_lock,yaw_lock;

  public:
    bool is_move=false;
    imu(PinName sda, PinName scl);

    int ADXL345_Initialize();
    int ADXL345_ReadData();
    int ADXL345_Calibration();
    int ITG3205_Initialize();
    int ITG3205_ReadData();
    int ITG3205_Calibration();
    int HMC5883L_Initialize();
    int HMC5883L_ReadData();

    void Mahony_Filter_Init(float ax,float ay,float az);
    void Mahony_Filter_Update();

    void Traditional_Linear_Filter_Init(float ax,float ay,float az);
    void Traditional_Linear_Filter_Update();

    float Get_Roll();
    float Get_Pitch();
    float Get_Yaw();
    float Get_Roll_Speed();
    float Get_Pitch_Speed();
    float Get_Yaw_Speed();
    float Get_ACC_X();
    float Get_ACC_Y();
    float Get_ACC_Z();

};

#endif
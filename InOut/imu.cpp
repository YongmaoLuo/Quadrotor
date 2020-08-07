#include <cstdio>
#include "mbed.h"
#include "imu.h"

imu::imu(PinName sda,PinName scl){
    this->i2c_imu=new I2C(sda,scl);
    this->i2c_imu->frequency(400000);
}

int imu::ADXL345_Initialize(){
    int ok=0;
    char write_data[2];
    // standby mode，设置寄存器POWER_CTL测量位为0,方便调试芯片其他参数
    write_data[0]=ADXL345_POWER_CTL;
    write_data[1]=0x00;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("set to standby mode failed!\n");
        return ok;
    }
    
    // 0禁用自测力，0SPI不需要理会，0中断高电平有效，0，1全分辨率模式，0右对齐，11范围为-16g->+16g
    write_data[0]=ADXL345_DATA_FORMAT;
    write_data[1]=0x0B;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if(ok!=0){
        printf("write to data format register failed!\n");
        return ok;
    }

    //设置寄存器BW_RATE速率为1600Hz
    write_data[0]=ADXL345_BW_RATE;
    write_data[1]=ADXL345_1600HZ;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("set data transmit rate failed!\n");
        return ok;
    }

    // 开始测量，设置寄存器POWER_CTL测量位为1
    write_data[0]=ADXL345_POWER_CTL;
    write_data[1]=0x08;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("begin measurement failed!\n");
        return ok;
    }
    return ok;
}

int imu::ADXL345_ReadData(){
    int ok=0;
    char read_data[6],register_address;
    memset(read_data, 0, sizeof(char[6]));
    //read DATA
    register_address=0x32;// DATAX0寄存器地址
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS, &register_address, true);//设置要读取的寄存器首地址
    if(ok!=0){
        printf("set read register failed!\n");
        return ok;
    }
    ok=i2c_imu->read(ADXL345_ADDRESS_8BITS, read_data, 6,true);
    if(ok!=0){
        printf("read data failed!\n");
        return ok;
    }
    
    double xt,yt,zt;
    xt=int16_t(int16_t(read_data[1])<<8|int16_t(read_data[0]))*ADXL345_RATIO;
    yt=int16_t(int16_t(read_data[3])<<8|int16_t(read_data[2]))*ADXL345_RATIO;
    zt=int16_t(int16_t(read_data[5])<<8|int16_t(read_data[4]))*ADXL345_RATIO;
    //printf("%f %f %f\n",xt,yt,zt);
    // 低通滤波
    this->acc_x=xt*ALPHA+this->acc_x*(1.0-ALPHA);
    this->acc_y=yt*ALPHA+this->acc_y*(1.0-ALPHA);
    this->acc_z=zt*ALPHA+this->acc_z*(1.0-ALPHA);
    return ok;
}

int imu::ADXL345_Calibration(){
    int ok=0;
    char write_data[2];
    int32_t average_read_x=0,average_read_y=0,average_read_z=0;// 储存读取的x,y,z平均值
    char read_data[6],register_address;
    memset(read_data, 0, sizeof(char[6]));
    
    for(int i=0;i<10;i++){ //读取10次确认飞静止且读数没有错误
        //读取原始数据
        register_address=0x32;// DATAX0寄存器地址
        ok=i2c_imu->write(ADXL345_ADDRESS_8BITS, &register_address, true);//设置要读取的寄存器首地址
        ok=i2c_imu->read(ADXL345_ADDRESS_8BITS, read_data, 6,true);
        if(ok!=0){
            while(1){printf("ADXL345 Read Data in Calibration failed!\n");}
        }
        average_read_x+=int16_t(int16_t(read_data[1])<<8|int16_t(read_data[0]));
        average_read_y=int16_t(int16_t(read_data[3])<<8|int16_t(read_data[2]));
        average_read_z=int16_t(int16_t(read_data[5])<<8|int16_t(read_data[4]));
    }
    //计算三轴平均读数
    average_read_x/=10;
    average_read_y/=10;
    average_read_z/=10;
    //计算偏移量
    average_read_x=-(average_read_x+2)/4;
    average_read_y=-(average_read_y+2)/4;
    average_read_z=-(average_read_z+2)/4;// 因为z轴的是向下的，测出的值小于-1，所以偏移量小于0，加上正向偏移量
    //写入偏移量
    // standby mode，设置寄存器POWER_CTL测量位为0,方便调试芯片其他参数
    write_data[0]=ADXL345_POWER_CTL;
    write_data[1]=0x00;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("set to standby mode failed!\n");
        return ok;
    }
    // 写入偏移量寄存器
    write_data[0]=ADXL345_OFFSETX;
    write_data[1]=int8_t(average_read_x);
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("write to offset_x failed!\n");
        return ok;
    }
    write_data[0]=ADXL345_OFFSETY;
    write_data[1]=int8_t(average_read_y);
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("write to offset_y failed!\n");
        return ok;
    }
    write_data[0]=ADXL345_OFFSETZ;
    write_data[1]=int8_t(average_read_z);
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("write to offset_z failed!\n");
        return ok;
    }
    // 开始测量，设置寄存器POWER_CTL测量位为1
    write_data[0]=ADXL345_POWER_CTL;
    write_data[1]=0x08;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("begin measurement failed!\n");
        return ok;
    }
    return ok;
}

int imu::ITG3205_Initialize(){
    int ok=0;
    char write_data[2];
    write_data[0]=ITG3205_SAMPLE_RATE_DIVIDER;
    write_data[1]=0x07;//被除数为7+1
    ok=i2c_imu->write(ITG3205_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("set sample rate divider failed!\n");
        return ok;
    }
    write_data[0]=ITG3205_DLPF_FS;
    write_data[1]=0x1E;//内部采样频率为1kHz，低通滤波器带宽为5Hz，设置全范围感知
    ok=i2c_imu->write(ITG3205_ADDRESS_8BITS,write_data,2);
    if(ok!=0){
        printf("set sample rate divider failed!\n");
        return ok;
    }
    return ok;
}

int imu::ITG3205_ReadData(){
    int ok=0;
    char read_data[6],register_address;
    memset(read_data, 0, sizeof(char[6]));
    //read DATA
    register_address=ITG3205_XH;// XOUTH寄存器地址
    ok=i2c_imu->write(ITG3205_ADDRESS_8BITS, &register_address, 1);//设置要读取的寄存器首地址
    if(ok!=0){
        printf("set read register failed!\n");
        return ok;
    }
    ok=i2c_imu->read(ITG3205_ADDRESS_8BITS, read_data, 6,true);
    if(ok!=0){
        printf("read data failed!\n");
        return ok;
    }
    
    this->gyr_x=int16_t(int16_t(read_data[0])<<8|int16_t(read_data[1]))/ITG3205_SENSITIVITY-this->gyr_offsetx;
    this->gyr_y=int16_t(int16_t(read_data[2])<<8|int16_t(read_data[3]))/ITG3205_SENSITIVITY-this->gyr_offsety;
    this->gyr_z=int16_t(int16_t(read_data[4])<<8|int16_t(read_data[5]))/ITG3205_SENSITIVITY-this->gyr_offsetz;
    //printf("%f %f %f\n",gyr_x,gyr_y,gyr_z);
    return ok;
}

int imu::ITG3205_Calibration(){
    int ok=0;
    char write_data[2],read_data[6],register_address;
    int32_t offset_x=0,offset_y=0,offset_z=0;
    for(int i=0;i<10;i++){
        //read DATA
        register_address=ITG3205_XH;// XOUTH寄存器地址
        ok=i2c_imu->write(ITG3205_ADDRESS_8BITS, &register_address, 1);//设置要读取的寄存器首地址
        ok=i2c_imu->read(ITG3205_ADDRESS_8BITS, read_data, 6,true);
        if(ok!=0){
            while(1){printf("ITG3205 read data in calibration failed!\n");}
        }
        offset_x+=int16_t(int16_t(read_data[0])<<8|int16_t(read_data[1]));
        offset_y+=int16_t(int16_t(read_data[2])<<8|int16_t(read_data[3]));
        offset_z+=int16_t(int16_t(read_data[4])<<8|int16_t(read_data[5]));
    }
    //取平均值
    offset_x/=10;
    offset_y/=10;
    offset_z/=10;
    this->gyr_offsetx=offset_x/ITG3205_SENSITIVITY;
    this->gyr_offsety=offset_y/ITG3205_SENSITIVITY;
    this->gyr_offsetz=offset_z/ITG3205_SENSITIVITY;
    return ok;
}

void imu::Mahony_Filter_Init(){
    float initial_roll, initial_pitch;
    float cos_roll, sin_roll, cos_pitch, sin_pitch;
    float mag_x, mag_y;
    float initial_hdg, cos_heading, sin_heading;

    initial_roll = atan2(-acc_y, -acc_z);
    initial_pitch = asin(acc_x);

    cos_roll = cosf(initial_roll);
    sin_roll = sinf(initial_roll);
    cos_pitch = cosf(initial_pitch);
    sin_pitch = sinf(initial_pitch);
    /*
    mag_x = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
    mag_y = my * cos_roll - mz * sin_roll; 
    initial_hdg = atan2f(-mag_y, mag_x);
    */
    initial_hdg=0;

    cos_roll = cosf(initial_roll * 0.5f);
    sin_roll = sinf(initial_roll * 0.5f);

    cos_pitch = cosf(initial_pitch * 0.5f);
    sin_pitch = sinf(initial_pitch * 0.5f);

    cos_heading = cosf(initial_hdg * 0.5f);
    sin_heading = sinf(initial_hdg * 0.5f);

    //初始化姿态四元数
    this->q0 = cos_roll * cos_pitch * cos_heading + sin_roll * sin_pitch * sin_heading;
    this->q1 = sin_roll * cos_pitch * cos_heading - cos_roll * sin_pitch * sin_heading;
    this->q2 = cos_roll * sin_pitch * cos_heading + sin_roll * cos_pitch * sin_heading;
    this->q3 = cos_roll * cos_pitch * sin_heading - sin_roll * sin_pitch * cos_heading;
}

void imu::Mahony_Filter_Update(){
    double gx,gy,gz,ax,ay,az;
    double vx,vy,vz;//上次姿态四元数解算得到的重力向量
    double ex,ey,ez;// 储存加速度计和陀螺仪在重力向量方面的误差
    gx=this->gyr_x;gy=this->gyr_y;gz=this->gyr_z;
    ax=this->acc_x;ay=this->acc_y;az=this->acc_z;
    
    double norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;ay /= norm;az /= norm;// 归一化
    
    // 利用之前的姿态四元数计算机体坐标系下的重力向量
    vx = 2*(this->q1*this->q3 - this->q0*this->q2);
    vy = 2*(this->q0*this->q1 + this->q2*this->q3);
    vz = this->q0*this->q0 - this->q1*this->q1 - this->q2*this->q2 + this->q3*this->q3;
    norm=sqrt(vx*vx+vy*vy+vz*vz);
    vx/=norm;vy/=norm;vz/=norm;// 归一化

    // 利用两向量之间的叉乘计算加速度计和陀螺仪测得的重力向量之间的误差
    // 而且由于估计姿态是由陀螺仪数据累计得到的，所以陀螺仪数据是有累计误差的，自然误差需要累加
    ex+=-vz*ay+vy*az;
    ey+=vz*ax-vx*az;
    ez+=-vy*ax+vx*ay;

    // 用误差更新误差积分
    this->exInt += ex*GYR_KI;
    this->eyInt += ey*GYR_KI;
    this->ezInt += ez*GYR_KI;

    //利用误差和误差积分（即PI）修正陀螺仪的测量数据
    gx += GYR_KP*ex + this->exInt;
    gy += GYR_KP*ey + this->eyInt;
    gz += GYR_KP*ez + this->ezInt;

    //利用四元数微分和其本身的关系，得到微分，然后将微分累加实时修正当前四元数
    this->q0 += (-this->q1*gx - this->q2*gy - this->q3*gz)*DT/2;
    this->q1 += (this->q0*gx + this->q2*gz - this->q3*gy)*DT/2;
    this->q2 += (this->q0*gy - this->q1*gz + this->q3*gx)*DT/2;
    this->q3 += (this->q0*gz + this->q1*gy - this->q2*gx)*DT/2;

    //将四元数转换为欧拉角
    this->roll=atan2(2*this->q2*this->q3 + 2*this->q0*this->q1, -2*this->q1*this->q1 - 2 *this->q2*this->q2 + 1)* 57.325;
    this->pitch=asin(-2*this->q1*this->q3 + 2*this->q0*this->q2)*57.325; 
    this->yaw=atan2(2 *this->q1 * this->q2 + 2 * this->q0 * this->q3, -2 * this->q2*this->q2 - 2 * this->q3* this->q3 + 1)* 57.3;
}

double imu::Get_Pitch(){
    return this->pitch;
}

double imu::Get_Roll(){
    return this->roll;
}

double imu::Get_Yaw(){
    return this->yaw;
}

double imu::Get_Roll_Speed(){
    return this->gyr_x;
}

double imu::Get_Pitch_Speed(){
    return this->gyr_y;
}

double imu::Get_Yaw_Speed(){
    return this->gyr_z;
}
#include <cstdio>
#include "mbed.h"
#include "imu.h"

imu::imu(PinName sda,PinName scl){
    this->i2c_imu=new I2C(sda,scl);
    this->i2c_imu->frequency(400000);
}

bool imu::ADXL345_Initialize(){
    bool ok=true;
    char write_data[2];
    // standby mode，设置寄存器POWER_CTL测量位为0,方便调试芯片其他参数
    write_data[0]=ADXL345_POWER_CTL;
    write_data[1]=0x00;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(!ok){
        printf("set to standby mode failed!\n");
        return ok;
    }
    
    // 0禁用自测力，0SPI不需要理会，0中断高电平有效，0，1全分辨率模式，0右对齐，11范围为-16g->+16g
    write_data[0]=ADXL345_DATA_FORMAT;
    write_data[1]=0x8B;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if(!ok){
        printf("write to data format register failed!\n");
        return ok;
    }

    //设置寄存器BW_RATE速率为1600Hz
    write_data[0]=ADXL345_BW_RATE;
    write_data[1]=ADXL345_1600HZ;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(!ok){
        printf("set data transmit rate failed!\n");
        return ok;
    }

    // 开始测量，设置寄存器POWER_CTL测量位为1
    write_data[0]=ADXL345_POWER_CTL;
    write_data[1]=0x08;// 第一行是寄存器地址，第二行是数据内容
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS,write_data,2);
    if(!ok){
        printf("begin measurement failed!\n");
        return ok;
    }
    return ok;
}

void imu::ADXL345_ReadData(){
    bool ok=true;
    char read_data[6],register_address;
    memset(read_data, 0, sizeof(char[6]));
    //read DATA
    register_address=0x32;// DATAX0寄存器地址
    ok=i2c_imu->write(ADXL345_ADDRESS_8BITS, &register_address, 1);//设置要读取的寄存器首地址
    if(!ok){
        printf("set read register failed!\n");
        return;
    }
    ok=i2c_imu->read(ADXL345_ADDRESS_8BITS, read_data, 6,true);
    if(!ok){
        printf("read data failed!\n");
        return;
    }
    
    double xt,yt,zt;
    xt=double(int16_t(read_data[1])<<8|int16_t(read_data[0]))*ADXL345_RATIO;
    yt=double(int16_t(read_data[3])<<8|int16_t(read_data[2]))*ADXL345_RATIO;
    zt=double(int16_t(read_data[5])<<8|int16_t(read_data[4]))*ADXL345_RATIO;
    printf("%f %f %f\n",xt,yt,zt);
    // 低通滤波
    this->acc_x=xt*ALPHA+this->acc_x*(1.0-ALPHA);
    this->acc_y=yt*ALPHA+this->acc_y*(1.0-ALPHA);
    this->acc_z=zt*ALPHA+this->acc_z*(1.0-ALPHA);
}

bool imu::ITG3205_Initialize(){
    bool ok=true;
    char write_data[2];
    write_data[0]=ITG3205_SAMPLE_RATE_DIVIDER;
    write_data[1]=0x07;//被除数为7+1
    ok=i2c_imu->write(ITG3205_ADDRESS_8BITS,write_data,2);
    if(!ok){
        printf("set sample rate divider failed!\n");
        return ok;
    }
    write_data[0]=ITG3205_DLPF_FS;
    write_data[1]=0x1E;//内部采样频率为1kHz，低通滤波器带宽为5Hz，设置全范围感知
    ok=i2c_imu->write(ITG3205_ADDRESS_8BITS,write_data,2);
    if(!ok){
        printf("set sample rate divider failed!\n");
        return ok;
    }
    return ok;
}

void imu::ITG3205_ReadData(){
    bool ok=true;
    char read_data[6],register_address;
    memset(read_data, 0, sizeof(char[6]));
    //read DATA
    register_address=ITG3205_XH;// XOUTH寄存器地址
    ok=i2c_imu->write(ITG3205_ADDRESS_8BITS, &register_address, 1);//设置要读取的寄存器首地址
    if(!ok){
        printf("set read register failed!\n");
        return;
    }
    ok=i2c_imu->read(ITG3205_ADDRESS_8BITS, read_data, 6,true);
    if(!ok){
        printf("read data failed!\n");
        return;
    }
    
    gyr_x=double(int16_t(read_data[0])<<8|int16_t(read_data[1]))/ITG3205_SENSITIVITY;
    gyr_y=double(int16_t(read_data[2])<<8|int16_t(read_data[3]))/ITG3205_SENSITIVITY;
    gyr_z=double(int16_t(read_data[4])<<8|int16_t(read_data[5]))/ITG3205_SENSITIVITY;
    printf("%f %f %f\n",gyr_x,gyr_y,gyr_z);
}
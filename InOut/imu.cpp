#include "imu.h"
#include "mbed.h"
#include <cstdio>

imu::imu(PinName sda, PinName scl) {
    this->i2c_imu = new I2C(sda, scl);
    this->i2c_imu->frequency(400000);
}

int imu::ADXL345_Initialize() {
    int ok = 0;
    char write_data[2];
    // standby mode，设置寄存器POWER_CTL测量位为0,方便调试芯片其他参数
    write_data[0] = ADXL345_POWER_CTL;
    write_data[1] = 0x00; // 第一行是寄存器地址，第二行是数据内容
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("set to standby mode failed!\n");
        return ok;
    }

    // 0禁用自测力，0SPI不需要理会，0中断高电平有效，0，1全分辨率模式，0右对齐，11范围为-16g->+16g
    write_data[0] = ADXL345_DATA_FORMAT;
    write_data[1] = 0x0B; // 第一行是寄存器地址，第二行是数据内容
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("write to data format register failed!\n");
        return ok;
    }

    //设置寄存器BW_RATE速率为1600Hz
    write_data[0] = ADXL345_BW_RATE;
    write_data[1] = ADXL345_1600HZ; // 第一行是寄存器地址，第二行是数据内容
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("set data transmit rate failed!\n");
        return ok;
    }

    // 开始测量，设置寄存器POWER_CTL测量位为1
    write_data[0] = ADXL345_POWER_CTL;
    write_data[1] = 0x08; // 第一行是寄存器地址，第二行是数据内容
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("begin measurement failed!\n");
        return ok;
    }
    return ok;
}

int imu::ADXL345_ReadData() {
    int ok = 0;
    char read_data[6], register_address;
    memset(read_data, 0, sizeof(char[6]));
    // read DATA
    register_address = 0x32; // DATAX0寄存器地址
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, &register_address,
                        true); //设置要读取的寄存器首地址
    if (ok != 0) {
        printf("set read register failed!\n");
        return ok;
    }
    ok = i2c_imu->read(ADXL345_ADDRESS_8BITS, read_data, 6, true);
    if (ok != 0) {
        printf("read data failed!\n");
        return ok;
    }

    uint16_t read_x_uint, read_y_uint, read_z_uint;

    double xt, yt, zt;

    //防止不同编译器对char的解释不同，我们采用判断的方法将原始数据扩展到16位
    read_x_uint = uint16_t(read_data[1]) << 8 | uint16_t(read_data[0]);
    read_y_uint = uint16_t(read_data[3]) << 8 | uint16_t(read_data[2]);
    read_z_uint = uint16_t(read_data[5]) << 8 | uint16_t(read_data[4]);
    // printf("x:%dy:%dz:%d\n",read_x_uint,read_y_uint,read_z_uint);

    //我们是13位都为有效数据
    if (read_x_uint > 0x0FFF) {
        read_x_uint = (~read_x_uint) + 1; //得到这个负数的相反数
        xt = -(int16_t(read_x_uint) * ADXL345_RATIO);
    } else {
        xt = double(double(read_x_uint) * ADXL345_RATIO);
    }
    if (read_y_uint > 0x0FFF) {
        read_y_uint = (~read_y_uint) + 1;
        yt = -int16_t(read_y_uint) * ADXL345_RATIO;
    } else {
        yt = int16_t(read_y_uint) * ADXL345_RATIO;
    }
    if (read_z_uint > 0x0FFF) {
        read_z_uint = (~read_z_uint) + 1;
        zt = -int16_t(read_z_uint) * ADXL345_RATIO;
    } else {
        zt = int16_t(read_z_uint) * ADXL345_RATIO;
    }

    this->acc_x=xt;
    this->acc_y=yt;
    this->acc_z=zt; 

    // printf("x:%dy:%dz:%d\n",read_x_uint,read_y_uint,read_z_uint);
    // printf("%f %f %f\n",xt,yt,zt);
     
     /* 
    // 低通滤波
    this->acc_x=xt*ALPHA+this->acc_x*(1.0-ALPHA);
    this->acc_y=yt*ALPHA+this->acc_y*(1.0-ALPHA);
    this->acc_z=zt*ALPHA+this->acc_z*(1.0-ALPHA);
*/
/* 
    //更新之前加速度计储存的矩阵
    this->before_acc_x[this->before_acc_point_x++]=xt;
    this->before_acc_y[this->before_acc_point_y++]=yt;
    this->before_acc_z[this->before_acc_point_z++]=zt;

    this->before_acc_point_x=this->before_acc_point_x%ACC_BEFORE_NUMBER;
    this->before_acc_point_y=this->before_acc_point_y%ACC_BEFORE_NUMBER;
    this->before_acc_point_z=this->before_acc_point_z%ACC_BEFORE_NUMBER;

    double aver_acc_x=0,aver_acc_y=0,aver_acc_z=0;
    for(int i=0;i<ACC_BEFORE_NUMBER;i++){
        aver_acc_x+=this->before_acc_x[i];
        aver_acc_y+=this->before_acc_y[i];
        aver_acc_z+=this->before_acc_z[i];
    }
    aver_acc_x/=ACC_BEFORE_NUMBER*1.0f;
    aver_acc_y/=ACC_BEFORE_NUMBER*1.0f;
    aver_acc_z/=ACC_BEFORE_NUMBER*1.0f;

    this->acc_x=aver_acc_x;
    this->acc_y=aver_acc_y;
    this->acc_z=aver_acc_z;
 
    //printf("acc: %f,%f,%f\n",acc_x,acc_y,acc_z);
    int is_positive_x=1,is_positive_y=1,is_positive_z=1;
    if(acc_x<0){
        is_positive_x=0;
    }
    if(acc_y<0){
        is_positive_y=0;
    }
    if(acc_z<0){
        is_positive_z=0;
    }
*/
    //printf("x:%d, y:%d, z:%d\n",is_positive_x,is_positive_y,is_positive_z);
    return ok;
}

int imu::ADXL345_Calibration() {
    int ok = 0;
    char write_data[2];
    double average_read_x = 0, average_read_y = 0,
            average_read_z = 0; // 储存读取的x,y,z平均值
    char read_data[6], register_address;
    memset(read_data, 0, sizeof(char[6]));
    Timer ACC_100Hz;
    ACC_100Hz.start();
    int count=0;
    while(1){
        if(ACC_100Hz.elapsed_time().count()>=10000){
            ACC_100Hz.reset();
            count++;
            //读取原始数据
            register_address = 0x32; // DATAX0寄存器地址
            ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, &register_address,
                                true); //设置要读取的寄存器首地址
            ok = i2c_imu->read(ADXL345_ADDRESS_8BITS, read_data, 6, true);
            if (ok != 0) {
                while (1) {
                    printf("ADXL345 Read Data in Calibration failed!\n");
                }
            }
            average_read_x += int16_t(read_data[1] << 8 | read_data[0]);
            average_read_y += int16_t(read_data[3] << 8 | read_data[2]);
            average_read_z += (int16_t(read_data[5] << 8 | read_data[4]));
            average_read_z+=256;
        }
        if(count>=50){
            ACC_100Hz.stop();
            break;
        }
    }
    
    //计算三轴平均读数
    average_read_x /= 50.0f;
    average_read_y /= 50.0f;
    average_read_z /= 50.0f;
    printf("%f, %f, %f\n",average_read_x,average_read_y,average_read_z);
    //计算偏移量
    average_read_x = int(-(average_read_x + 2) / 4);
    average_read_y = int(-(average_read_y + 2) / 4);
    average_read_z =
        int(-(average_read_z + 2) /4); // 因为z轴的是向下的，测出的值小于-1，所以偏移量小于0，加上正向偏移量
    
    //四舍五入


    //写入偏移量
    // standby mode，设置寄存器POWER_CTL测量位为0,方便调试芯片其他参数
    write_data[0] = ADXL345_POWER_CTL;
    write_data[1] = 0x00; // 第一行是寄存器地址，第二行是数据内容
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("set to standby mode failed!\n");
        return ok;
    }
    // 写入偏移量寄存器
    write_data[0] = ADXL345_OFFSETX;
    write_data[1] = int8_t(average_read_x);
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("write to offset_x failed!\n");
        return ok;
    }
    write_data[0] = ADXL345_OFFSETY;
    write_data[1] = int8_t(average_read_y);
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("write to offset_y failed!\n");
        return ok;
    }
    write_data[0] = ADXL345_OFFSETZ;
    write_data[1] = int8_t(average_read_z);
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("write to offset_z failed!\n");
        return ok;
    }
    // 开始测量，设置寄存器POWER_CTL测量位为1
    write_data[0] = ADXL345_POWER_CTL;
    write_data[1] = 0x08; // 第一行是寄存器地址，第二行是数据内容
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("begin measurement failed!\n");
        return ok;
    }
    printf("The Calibration:%f,%f,%f",average_read_x,average_read_y,average_read_z);   
    return ok;
}

int imu::ITG3205_Initialize() {
    int ok = 0;
    char write_data[2];
    write_data[0] = ITG3205_SAMPLE_RATE_DIVIDER;
    write_data[1] = 0x00; //被除数为1+1
    ok = i2c_imu->write(ITG3205_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("set sample rate divider failed!\n");
        return ok;
    }
    write_data[0] = ITG3205_DLPF_FS;
    write_data[1] =
        0x1E; //内部采样频率为1kHz，低通滤波器带宽为5Hz，设置全范围感知
    ok = i2c_imu->write(ITG3205_ADDRESS_8BITS, write_data, 2);
    if (ok != 0) {
        printf("set sample rate divider failed!\n");
        return ok;
    }
    return ok;
}

int imu::ITG3205_ReadData() {
    int ok = 0;
    char read_data[6], register_address;
    memset(read_data, 0, sizeof(char[6]));
    // read DATA
    register_address = ITG3205_XH; // XOUTH寄存器地址
    ok = i2c_imu->write(ITG3205_ADDRESS_8BITS, &register_address,
                        1); //设置要读取的寄存器首地址
    if (ok != 0) {
        printf("set read register failed!\n");
        return ok;
    }
    ok = i2c_imu->read(ITG3205_ADDRESS_8BITS, read_data, 6, true);
    if (ok != 0) {
        printf("read data failed!\n");
        return ok;
    }

    double xt,yt,zt;
    xt =
        int16_t(read_data[0] << 8 | read_data[1]) / ITG3205_SENSITIVITY -
        this->gyr_offsetx;
    yt =
        int16_t(read_data[2] << 8 | read_data[3]) / ITG3205_SENSITIVITY -
        this->gyr_offsety;
    zt =
        int16_t(read_data[4] << 8 | read_data[5]) / ITG3205_SENSITIVITY -
        this->gyr_offsetz;
    
    this->gyr_x=xt;
    this->gyr_y=yt;
    this->gyr_z=zt;
/* 
    //更新之前加速度计储存的矩阵
    this->before_gyr_x[this->before_gyr_point_x++]=xt;
    this->before_gyr_y[this->before_gyr_point_y++]=yt;
    this->before_gyr_z[this->before_gyr_point_z++]=zt;

    this->before_gyr_point_x=this->before_gyr_point_x%GYR_BEFORE_NUMBER;
    this->before_gyr_point_y=this->before_gyr_point_y%GYR_BEFORE_NUMBER;
    this->before_gyr_point_z=this->before_gyr_point_z%GYR_BEFORE_NUMBER;

    double aver_gyr_x=0,aver_gyr_y=0,aver_gyr_z=0;
    for(int i=0;i<GYR_BEFORE_NUMBER;i++){
        aver_gyr_x+=this->before_gyr_x[i];
        aver_gyr_y+=this->before_gyr_y[i];
        aver_gyr_z+=this->before_gyr_z[i];
    }
    aver_gyr_x/=GYR_BEFORE_NUMBER*1.0f;
    aver_gyr_y/=GYR_BEFORE_NUMBER*1.0f;
    aver_gyr_z/=GYR_BEFORE_NUMBER*1.0f;

    this->gyr_x=aver_gyr_x;
    this->gyr_y=aver_gyr_y;
    this->gyr_z=aver_gyr_z;
 */
    //printf("%f %f %f\n",gyr_x,gyr_y,gyr_z);
    return ok;
}

int imu::ITG3205_Calibration() {
    int ok = 0;
    char read_data[6], register_address;
    int32_t offset_x = 0, offset_y = 0, offset_z = 0;
    for (int i = 0; i < 10; i++) {
        // read DATA
        register_address = ITG3205_XH; // XOUTH寄存器地址
        ok = i2c_imu->write(ITG3205_ADDRESS_8BITS, &register_address,
                            1); //设置要读取的寄存器首地址
        ok = i2c_imu->read(ITG3205_ADDRESS_8BITS, read_data, 6, true);
        if (ok != 0) {
            while (1) {
                printf("ITG3205 read data in calibration failed!\n");
            }
        }
        offset_x += int16_t(read_data[0] << 8 | read_data[1]);
        offset_y += int16_t(read_data[2] << 8 | read_data[3]);
        offset_z += int16_t(read_data[4] << 8 | read_data[5]);
    }
    //取平均值
    offset_x /= 10;
    offset_y /= 10;
    offset_z /= 10;
    this->gyr_offsetx = offset_x / ITG3205_SENSITIVITY;
    this->gyr_offsety = offset_y / ITG3205_SENSITIVITY;
    this->gyr_offsetz = offset_z / ITG3205_SENSITIVITY;
    return ok;
}

void imu::Mahony_Filter_Init(double ax,double ay,double az) {
    double initial_roll, initial_pitch;
    double cos_roll, sin_roll, cos_pitch, sin_pitch;
    double initial_hdg, cos_heading, sin_heading;

    double norm = sqrt(ax * ax + ay * ay + az * az);
    double tan_roll=ay/az;
    initial_roll = atan(tan_roll);
    initial_pitch = asin(ax / norm);
    /* 
    while(1)
    printf("init_roll:%f\n",initial_roll);
*/
    cos_roll = cosf(initial_roll);
    sin_roll = sinf(initial_roll);
    cos_pitch = cosf(initial_pitch);
    sin_pitch = sinf(initial_pitch);
    /*
    mag_x = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll *
    sin_pitch; mag_y = my * cos_roll - mz * sin_roll; initial_hdg =
    atan2f(-mag_y, mag_x);
    */
    initial_hdg = 0;

    cos_roll = cosf(initial_roll * 0.5f);
    sin_roll = sinf(initial_roll * 0.5f);

    cos_pitch = cosf(initial_pitch * 0.5f);
    sin_pitch = sinf(initial_pitch * 0.5f);

    cos_heading = cosf(initial_hdg * 0.5f);
    sin_heading = sinf(initial_hdg * 0.5f);

    //初始化姿态四元数
    this->q0 =
        cos_roll * cos_pitch * cos_heading + sin_roll * sin_pitch * sin_heading;
    this->q1 =
        sin_roll * cos_pitch * cos_heading - cos_roll * sin_pitch * sin_heading;
    this->q2 =
        cos_roll * sin_pitch * cos_heading + sin_roll * cos_pitch * sin_heading;
    this->q3 =
        cos_roll * cos_pitch * sin_heading - sin_roll * sin_pitch * cos_heading;
    
}

void imu::Mahony_Filter_Update() {
    double gx, gy, gz, ax, ay, az;
    
    double gyr_kp = GYR_KP, gyr_ki = GYR_KI, halfdt = 1.0 / (2.0*SAMPLE_RATE);
    double m_q0 = this->q0, m_q1 = this->q1, m_q2 = this->q2, m_q3 = this->q3;

    gx = this->gyr_x;
    gy = this->gyr_y;
    gz = this->gyr_z;
    ax = this->acc_x;
    ay = this->acc_y;
    az = this->acc_z;

    if(ax==0.0f&&ay==0.0f&&az==0.0f){
        return;
    }

    double norm = sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm; // 归一化
    
    printf("a vector:%f %f %f\n",ax,ay,az);
    
    double halfvx, halfvy, halfvz; //上次姿态四元数解算得到的重力向量
    // 利用之前的姿态四元数计算机体坐标系下的重力向量
    halfvx = (m_q1 * m_q3 - m_q0 * m_q2);
    halfvy = (m_q0 * m_q1 + m_q2 * m_q3);
    halfvz = (m_q0 * m_q0 - m_q1*m_q1-m_q2*m_q2 + m_q3 * m_q3)*0.5f;
     
    printf("v: %f,%f,%f\n",halfvx,halfvy,halfvz);
    
    double halfex,halfey,halfez;
    // 利用两向量之间的叉乘计算加速度计和陀螺仪测得的重力向量之间的误差
    // 这个误差用于每次有误差的时候进行姿态修正，所以不需要进行累加
    halfex = halfvz * ay - halfvy * az;
    halfey = -halfvz * ax + halfvx * az;
    halfez = halfvy * ax - halfvx * ay;
    
    //printf("e: %f,%f,%f\n",ex,ey,ez);
    
    // 用误差更新误差积分
    if(gyr_ki>0.0f){
        this->exInt += halfex * gyr_ki*(1.0f / SAMPLE_RATE);
        this->eyInt += halfey * gyr_ki*(1.0f / SAMPLE_RATE);
        this->ezInt += halfez * gyr_ki*(1.0f / SAMPLE_RATE);
    }else{
        this->exInt=0.0f;
        this->eyInt=0.0f;
        this->ezInt=0.0f;
    }

 /* 
    if(this->is_move==false){
        //利用误差和误差积分（即PI）修正陀螺仪的测量数据
        //并且增大KP值使其迅速收敛
        gx += ex*300;
        gy += ey*300;
        gz += ez*300;

        this->exInt=0;
        this->eyInt=0;
        this->ezInt=0;
    }else{
        //利用误差和误差积分（即PI）修正陀螺仪的测量数据

        gx += gyr_kp * ex + this->exInt;
        gy += gyr_kp * ey + this->eyInt;
        gz += gyr_kp * ez + this->ezInt;
    }
*/
        gx += gyr_kp * halfex + this->exInt;
        gy += gyr_kp * halfey + this->eyInt;
        gz += gyr_kp * halfez + this->ezInt;

        gx *= halfdt;
        gy*=halfdt;
        gz*=halfdt;
    //printf("g: %f,%f,%f\n",gx,gy,gz);

    //利用四元数微分和其本身的关系，得到微分，然后将微分累加实时修正当前四元数
    // printf("q0: %f,halfdt: %f\n",q0,halfdt);
    double dq0, dq1, dq2, dq3;
    dq0 = (-m_q1 * gx - m_q2 * gy - m_q3 * gz);
    dq1 = (m_q0 * gx + m_q2 * gz - m_q3 * gy);
    dq2 = (m_q0 * gy - m_q1 * gz + m_q3 * gx);
    dq3 = (m_q0 * gz + m_q1 * gy - m_q2 * gx);
    m_q0 += dq0;
    m_q1 += dq1;
    m_q2 += dq2;
    m_q3 += dq3;
    // printf("q: %f,%f,%f,%f\n",m_q0,m_q1,m_q2,m_q3);

    //将四元数归一化
    norm = sqrt(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
    m_q0 /= norm;
    m_q1 /= norm;
    m_q2 /= norm;
    m_q3 /= norm;

    //printf("q: %f,%f,%f,%f\n",m_q0,m_q1,m_q2,m_q3);

    double temp_roll, temp_pitch, temp_yaw;
    //将四元数转换为欧拉角
    double tan_phi = (2.0f * m_q2 * m_q3 + 2.0f * m_q0 * m_q1) /(-2.0f * m_q1 * m_q1 - 2.0f * m_q2 * m_q2 + 1.0f);
    temp_roll = atan(tan_phi) * 57.296f;
    double sin_theta = 2.0f * m_q0 * m_q2 - 2.0f * m_q1 * m_q3;
    temp_pitch = asin(sin_theta) * 57.296f;
    double tan_psi=(2.0f * m_q1 * m_q2 + 2.0f * m_q0 * m_q3)/(-2.0f * m_q2 * m_q2 - 2.0f * m_q3 * m_q3 + 1.0f);
    temp_yaw = atan(tan_psi) *57.296f;
    if (temp_roll == -HUGE_VAL || temp_pitch == -HUGE_VAL ||
        temp_yaw == -HUGE_VAL) {
        // 说明数据计算出现错误，不更新四元数，进行回滚
        printf("rollback quanternion\n");
        this->exInt = 0;
        this->eyInt = 0;
        this->ezInt = 0;
    } else {
        //更新储存的四元数
        this->q0 = m_q0;
        this->q1 = m_q1;
        this->q2 = m_q2;
        this->q3 = m_q3;

        //更新欧拉角
        this->roll = temp_roll;
        this->pitch = temp_pitch;
        this->yaw = temp_yaw;
    }

    // printf("q: %f,%f,%f,%f\n",m_q0,m_q1,m_q2,m_q3);
}

double imu::Get_Pitch() { return this->pitch; }

double imu::Get_Roll() { return this->roll; }

double imu::Get_Yaw() { return this->yaw; }

double imu::Get_Roll_Speed() { return this->gyr_x; }

double imu::Get_Pitch_Speed() { return this->gyr_y; }

double imu::Get_Yaw_Speed() { return this->gyr_z; }

double imu::Get_ACC_X(){return this->acc_x;}

double imu::Get_ACC_Y(){return this->acc_y;}

double imu::Get_ACC_Z(){return this->acc_z;}

void imu::Traditional_Linear_Filter_Init(double ax,double ay,double az){
    double norm = sqrt(ax * ax + ay * ay + az * az);
    double tan_roll=ay/az;
    this->roll = atan(tan_roll);
    this->pitch = asin(ax / norm);
    this->yaw=0;
}
void imu::Traditional_Linear_Filter_Update(){
    //互补滤波
    double ax=this->acc_x,ay=this->acc_y,az=this->acc_z;
    double norm = sqrt(ax * ax + ay * ay + az * az);
    double tan_roll=ay/az;
    double acc_roll=atan(tan_roll);
    double acc_pitch=atan(ax/norm);

    this->roll=(TAO/(TAO+1.0f/SAMPLE_RATE))*(this->last_roll+1.0f/SAMPLE_RATE*this->gyr_x)+(1.0f/SAMPLE_RATE/(TAO+1.0f/SAMPLE_RATE))*acc_roll* 57.296f;
    this->pitch=(TAO/(TAO+1.0f/SAMPLE_RATE))*(this->last_pitch+1.0f/SAMPLE_RATE*this->gyr_y)+(1.0f/SAMPLE_RATE/(TAO+1.0f/SAMPLE_RATE))*acc_pitch* 57.296f;
    this->yaw=this->last_yaw+1.0f/SAMPLE_RATE*this->gyr_z* 57.296f;

    // 滑动均值滤波
    this->last_roll=this->roll;
    this->last_pitch=this->pitch;
    this->last_yaw=this->yaw;
}
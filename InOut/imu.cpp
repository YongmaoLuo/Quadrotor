#include "imu.h"
#include "mbed.h"
#include <cstring>

float Invert_Sqrt(float x);

imu::imu(PinName sda, PinName scl) {
    acc_data[6]=0;gyr_data[6]=0;
    this->q0=1;this->q1=0;this->q2=0;this->q3=0;
    gyr_offsetx = gyr_offsety=gyr_offsetz=0;
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
    write_data[1] = ADXL345_800HZ; // 第一行是寄存器地址，第二行是数据内容
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
    char register_address;
    // read DATA
    register_address = 0x32; // DATAX0寄存器地址
    ok = i2c_imu->write(ADXL345_ADDRESS_8BITS, &register_address,true); //设置要读取的寄存器首地址
    acc_lock.lock();
    ok = i2c_imu->read(ADXL345_ADDRESS_8BITS, acc_data, 6, true);
    acc_lock.unlock();
    
    return ok;
}

int imu::ADXL345_Calibration() {
    int ok = 0;
    char write_data[2];
    float average_read_x = 0, average_read_y = 0,
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
            average_read_z += 256;
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
    //printf("%f, %f, %f\n",average_read_x,average_read_y,average_read_z);
    //计算偏移量同时四舍五入
    average_read_x = int(-(average_read_x + 2) / 4);
    average_read_y = int(-(average_read_y + 2) / 4);
    average_read_z =int(-(average_read_z + 2) /4); // 因为z轴的是向下的，测出的值小于-1，所以偏移量小于0，加上正向偏移量
    

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
    //printf("The Calibration:%f,%f,%f",average_read_x,average_read_y,average_read_z);   
    return ok;
}

int imu::ITG3205_Initialize() {
    int ok = 0;
    char write_data[2];
    write_data[0] = ITG3205_SAMPLE_RATE_DIVIDER;
    write_data[1] = 0x00; //被除数为1+0
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
    char register_address;
    // read DATA
    register_address = ITG3205_XH; // XOUTH寄存器地址
    ok = i2c_imu->write(ITG3205_ADDRESS_8BITS, &register_address,1); //设置要读取的寄存器首地址
    ok = i2c_imu->read(ITG3205_ADDRESS_8BITS, gyr_data, 6, true);
    
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

void imu::Mahony_Filter_Init(float ax,float ay,float az) {
    float initial_roll, initial_pitch;
    float cos_roll, sin_roll, cos_pitch, sin_pitch;
    float initial_hdg, cos_heading, sin_heading;

    // can be vindicated right by math
    float norm = Invert_Sqrt(ax * ax + ay * ay + az * az);
    float tan_roll=-ay/az;
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
/*     
    while(1){
        printf("%f, %f, %f, %f\n",q0,q1,q2,q3);
        if(q0>0){
            printf("q0:1, ");
        }else{
            printf("q0:0, ");
        }

        if(q1>0){
            printf("q1:1, ");
        }else{
            printf("q1:0, ");
        }

        if(q2>0){
            printf("q2:1, ");
        }else{
            printf("q2:0, ");
        }

        if(q3>0){
            printf("q3:1, \n");
        }else{
            printf("q3:0, \n");
        }
    }
   */ 
}


void imu::Mahony_Filter_Update() {
    char acc_raw[7],gyr_raw[7];
    float ax,ay,az,gx,gy,gz;
    
    float gyr_kp = GYR_KP, gyr_ki = GYR_KI, dt = 1.0 / (SAMPLE_RATE);
    float m_q0 = q0, m_q1 = q1, m_q2 = q2, m_q3 = q3;

    acc_lock.lock();
    std::strcpy(acc_raw,acc_data);
    acc_lock.unlock();

    gyr_lock.lock();
    std::strcpy(gyr_raw,gyr_data);
    gyr_lock.unlock();

    ax = int16_t(acc_raw[1] << 8 | acc_raw[0])*ADXL345_RATIO;
    ay = int16_t(acc_raw[3] << 8 | acc_raw[2])*ADXL345_RATIO;
    az = int16_t(acc_raw[5] << 8 | acc_raw[4])*ADXL345_RATIO;

    gx =int16_t(gyr_raw[0] << 8 | gyr_raw[1]) / ITG3205_SENSITIVITY - gyr_offsetx;
    gy =int16_t(gyr_raw[2] << 8 | gyr_raw[3]) / ITG3205_SENSITIVITY - gyr_offsety;
    gz =int16_t(gyr_raw[4] << 8 | gyr_raw[5]) / ITG3205_SENSITIVITY - gyr_offsetz;

    if(ax==0.0f&&ay==0.0f&&az==0.0f){
        return;
    }

    float norm = Invert_Sqrt(ax * ax + ay * ay + az * az);
    //float norm = sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm; // 归一化
    
    //printf("a vector:%f %f %f\n",ax,ay,az);
    
    float vx, vy, vz; //上次姿态四元数解算得到的重力向量
    // 利用之前的姿态四元数计算机体坐标系下的重力向量
    vx = 2*(m_q1 * m_q3 - m_q0 * m_q2);
    vy = 2*(m_q0 * m_q1 + m_q2 * m_q3);
    vz = m_q0 * m_q0 - m_q1 * m_q1-m_q2 * m_q2 + m_q3 * m_q3;
     
    //printf("v: %f,%f,%f\n",vx,vy,vz);
    
    float ex,ey,ez;
    // 利用两向量之间的叉乘计算加速度计和陀螺仪测得的重力向量之间的误差
    // 这个误差用于每次有误差的时候进行姿态修正，所以不需要进行累加
    ex = vz * ay - vy * az;
    ey = vx * az - vz * ax;
    ez = vy * ax - vx * ay;
    
    //printf("e: %f,%f,%f\n",ex,ey,ez);
    
    // 用误差更新误差积分
    if(gyr_ki>0.0f){
        exInt += ex * gyr_ki*(1.0f / SAMPLE_RATE);
        eyInt += ey * gyr_ki*(1.0f / SAMPLE_RATE);
        ezInt += ez * gyr_ki*(1.0f / SAMPLE_RATE);
    }else{
        exInt=0.0f;
        eyInt=0.0f;
        ezInt=0.0f;
    }

    gx = gx + gyr_kp * ex + exInt;
    gy = gy + gyr_kp * ey + eyInt;
    gz = gz + gyr_kp * ez + ezInt;

    //printf("g: %f,%f,%f\n",gx,gy,gz);

    gx = gx * dt*0.5f;
    gy = gy * dt*0.5f;
    gz = gz * dt*0.5f;
    //利用四元数微分和其本身的关系，得到微分，然后将微分累加实时修正当前四元数
    // printf("q0: %f,halfdt: %f\n",q0,halfdt);
    float dq0, dq1, dq2, dq3;
    dq0 = (-m_q1 * gx - m_q2 * gy - m_q3 * gz);
    dq1 = (m_q0 * gx + m_q2 * gz - m_q3 * gy);
    dq2 = (m_q0 * gy - m_q1 * gz + m_q3 * gx);
    dq3 = (m_q0 * gz + m_q1 * gy - m_q2 * gx);

    m_q0+=dq0;
    m_q1+=dq1;
    m_q2+=dq2;
    m_q3+=dq3;

/* 
    //四元数的二阶递推算法
    float delta = (dt*gx) * (dt*gx) + (dt*gy) * (dt*gy) + (dt*gz) * (dt*gz);
    float tmp0 = (1.0f - delta / 8.0f) * m_q0 + 0.5f * (-m_q1*gx - m_q2*gy - m_q3*gz) * dt;
    float tmp1 = (1.0f - delta / 8.0f) * m_q1 + 0.5f * ( m_q0*gx + m_q2*gz - m_q3*gy) * dt;
    float tmp2 = (1.0f - delta / 8.0f) * m_q2 + 0.5f * ( m_q0*gy - m_q1*gz + m_q3*gx) * dt;
    float tmp3 = (1.0f - delta / 8.0f) * m_q3 + 0.5f * ( m_q0*gz + m_q1*gy - m_q2*gx) * dt;
    m_q0 = tmp0;
    m_q1 = tmp1;
    m_q2 = tmp2;
    m_q3 = tmp3;
*/
    /* 
    printf("q: %f,%f,%f,%f\n",m_q0,m_q1,m_q2,m_q3);
    if(m_q0>0){
        printf("q0:1, ");
    }else{
        printf("q0:0, ");
    }
    if(m_q1>0){
        printf("q1:1, ");
    }else{
        printf("q1:0, ");
    }
    if(m_q2>0){
        printf("q2:1, ");
    }else{
        printf("q2:0, ");
    }

    if(m_q3>0){
        printf("q3:1, \n");
    }else{
        printf("q3:0, \n");
    }
*/
    //将四元数归一化
    norm = Invert_Sqrt(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
    m_q0 *= norm;
    m_q1 *= norm;
    m_q2 *= norm;
    m_q3 *= norm;

    //printf("q: %f,%f,%f,%f\n",m_q0,m_q1,m_q2,m_q3);

    float temp_roll, temp_pitch, temp_yaw;
    //将四元数转换为欧拉角
    //float tan_phi = (2.0f * m_q2 * m_q3 + 2.0f * m_q0 * m_q1) /(-2.0f * m_q1 * m_q1 - 2.0f * m_q2 * m_q2 + 1.0f);
    temp_roll = atan2(2.0f * m_q2 * m_q3 + 2.0f * m_q0 * m_q1, (-2.0f * m_q1 * m_q1 - 2.0f * m_q2 * m_q2 + 1.0f)) * 57.296f;
    float sin_theta = 2.0f * m_q0 * m_q2 - 2.0f * m_q1 * m_q3;
    temp_pitch = asin(sin_theta) * 57.296f;
    //float tan_psi=(2.0f * m_q1 * m_q2 + 2.0f * m_q0 * m_q3)/(-2.0f * m_q2 * m_q2 - 2.0f * m_q3 * m_q3 + 1.0f);
    temp_yaw = atan2((2.0f * m_q1 * m_q2 + 2.0f * m_q0 * m_q3),(-2.0f * m_q2 * m_q2 - 2.0f * m_q3 * m_q3 + 1.0f)) *57.296f;

    //更新储存的四元数
    q0 = m_q0;
    q1 = m_q1;
    q2 = m_q2;
    q3 = m_q3;

    //更新欧拉角
    roll_lock.lock();pitch_lock.lock();yaw_lock.lock();
    roll = temp_roll;pitch = temp_pitch;yaw = temp_yaw;
    roll_lock.unlock();pitch_lock.unlock();yaw_lock.unlock();

    //printf("%f, %f, %f\n",temp_roll,temp_pitch,temp_yaw);
    // printf("q: %f,%f,%f,%f\n",m_q0,m_q1,m_q2,m_q3);
}

float imu::Get_Pitch() { 
    pitch_lock.lock();
    float result=this->pitch;
    pitch_lock.unlock();
    return result; 
}

float imu::Get_Roll() { 
    roll_lock.lock();
    float result=this->roll;
    roll_lock.unlock();
    return result;
}

float imu::Get_Yaw() { 
    yaw_lock.lock();
    float result=this->yaw;
    yaw_lock.unlock();
    return result; 
}

float imu::Get_Roll_Speed() { 
    char temp[2];
    gyr_lock.lock();
    temp[0]=gyr_data[0];
    temp[1]=gyr_data[1];
    gyr_lock.unlock();
    float result=int16_t(temp[0]<<8|temp[1])/ITG3205_SENSITIVITY-gyr_offsetx;
    return result; 
}

float imu::Get_Pitch_Speed() {
    char temp[2]; 
    gyr_lock.lock();
    temp[0]=gyr_data[2];
    temp[1]=gyr_data[3];
    gyr_lock.unlock();
    float result=int16_t(temp[0]<<8|temp[1])/ITG3205_SENSITIVITY-gyr_offsetx;
    return result; 
}

float imu::Get_Yaw_Speed() { 
    char temp[2]; 
    gyr_lock.lock();
    temp[0]=gyr_data[4];
    temp[1]=gyr_data[5];
    gyr_lock.unlock();
    float result=int16_t(temp[0]<<8|temp[1])/ITG3205_SENSITIVITY-gyr_offsetx;
    return result; 
}

float imu::Get_ACC_X(){
    char temp[2]; 
    acc_lock.lock();
    temp[0]=acc_data[0];
    temp[1]=acc_data[1];
    acc_lock.unlock();
    float result=int16_t(temp[1]<<8|temp[0])*ADXL345_RATIO;
    return result; 
}

float imu::Get_ACC_Y(){
    char temp[2]; 
    acc_lock.lock();
    temp[0]=acc_data[2];
    temp[1]=acc_data[3];
    acc_lock.unlock();
    float result=int16_t(temp[1]<<8|temp[0])*ADXL345_RATIO;
    return result; 
}

float imu::Get_ACC_Z(){
    char temp[2]; 
    acc_lock.lock();
    temp[0]=acc_data[4];
    temp[1]=acc_data[5];
    acc_lock.unlock();
    float result=int16_t(temp[1]<<8|temp[0])*ADXL345_RATIO;
    return result; 
}

float Invert_Sqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
/* 
void imu::Traditional_Linear_Filter_Init(float ax,float ay,float az){
    
    float norm = sqrt(ax * ax + ay * ay + az * az);
    float tan_roll=-ay/az;
    this->roll = atan(tan_roll);
    this->pitch = asin(ax / norm);
    this->yaw=0;
}
void imu::Traditional_Linear_Filter_Update(){
    //互补滤波
    acc_lock.lock();
    float ax=this->acc_x,ay=this->acc_y,az=this->acc_z;
    acc_lock.unlock();
    float norm = sqrt(ax * ax + ay * ay + az * az);
    float tan_roll=ay/az;
    float acc_roll=atan(tan_roll);
    float acc_pitch=asin(ax/norm);

    this->roll=(TAO/(TAO+1.0f/SAMPLE_RATE))*(this->last_roll+1.0f/SAMPLE_RATE*this->gyr_x)+(1.0f/SAMPLE_RATE/(TAO+1.0f/SAMPLE_RATE))*acc_roll* 57.296f;
    this->pitch=(TAO/(TAO+1.0f/SAMPLE_RATE))*(this->last_pitch+1.0f/SAMPLE_RATE*this->gyr_y)+(1.0f/SAMPLE_RATE/(TAO+1.0f/SAMPLE_RATE))*acc_pitch* 57.296f;
    this->yaw=this->last_yaw+1.0f/SAMPLE_RATE*this->gyr_z* 57.296f;

    // 滑动均值滤波
    this->last_roll=this->roll;
    this->last_pitch=this->pitch;
    this->last_yaw=this->yaw;
}
*/
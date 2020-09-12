#include "command.h"
#include "mbed.h"

ppm_pwm_command::ppm_pwm_command(PinName in1, PinName out1, PinName out2,
                                 PinName out3, PinName out4) {
    this->ppm_in = new InterruptIn(in1, PullNone);
    this->current_channel = 0;
    this->state = false;

    this->motor1=new PwmOut(out1);
    this->motor2=new PwmOut(out2);
    this->motor3=new PwmOut(out3);
    this->motor4=new PwmOut(out4);

    this->motor1->period_us(5000);
    this->motor2->period_us(5000);
    this->motor3->period_us(5000);
    this->motor4->period_us(5000);
    /* // 不能在代码中直接放入校准代码，虽然省事，但是容易在飞行器运行过程中再次初始化时出现意外。
    Timer max_cali;
    max_cali.start();
    while(max_cali.elapsed_time().count()<=3000000){
    this->motor1->pulsewidth_us(2000);
    this->motor2->pulsewidth_us(2000);
    this->motor3->pulsewidth_us(2000);
    this->motor4->pulsewidth_us(2000);
    }
    max_cali.reset();
    while(max_cali.elapsed_time().count()<=3000000){
        this->motor1->pulsewidth_us(1000);
        this->motor2->pulsewidth_us(1000);
        this->motor3->pulsewidth_us(1000);
        this->motor4->pulsewidth_us(1000);
    }
    max_cali.stop();
    */
    Timer max_cali;
    max_cali.start();
    while(max_cali.elapsed_time().count()<=6000000){
        this->motor1->pulsewidth_us(1000);
        this->motor2->pulsewidth_us(1000);
        this->motor3->pulsewidth_us(1000);
        this->motor4->pulsewidth_us(1000);
    }
    max_cali.stop();
}

void ppm_pwm_command::Receive_PPM() {
    // 保证从第一个上升沿开始计数，否则第一个计时内容不准
    int ppm_num = this->ppm_in->read();
    // 之所以要检测下降沿是要确保之后的变化一定上升沿
    while (ppm_num) {
        ppm_num = this->ppm_in->read();
    }
    while (!ppm_num) {
        ppm_num = this->ppm_in->read();
    }
    this->timer_ppm.start();
    while(true){
        
        // 之所以要检测下降沿是要确保之后的变化一定上升沿
        while (ppm_num) {
            ppm_num = this->ppm_in->read();
        }
        while (!ppm_num) {
            ppm_num = this->ppm_in->read();
        }

        // 跳出循环说明收到高电平，此时是上升沿
        this->Store_Channel();
        //ppm_in->rise(&this->Store_Channel);
    }
}

void ppm_pwm_command::Store_Channel() {
    int64_t time =timer_ppm.elapsed_time().count(); // 不能让timer停下来，否则就是0
    timer_ppm.reset();
    if (time > 3000) { // 说明刚刚经过一个周期末尾的空白阶段
        if (current_channel == PPM_CHANNELS) {
            state = true;
            //printf("%lld %lld %lld %lld\n",channels[0],channels[1],channels[2],channels[3]);
        }
        current_channel = 0;
    } else if (current_channel >= PPM_CHANNELS) {
        printf("system channel numbers are small!\n");
        current_channel = 0;
    } else {
        ch_lock[current_channel].lock();
        channels[current_channel] = time;
        ch_lock[current_channel].unlock();
        current_channel = current_channel + 1;
    }
}

double ppm_pwm_command::Get_Roll_Command(){//输出滚转角
    double result;
    ch_lock[0].lock();
    result=(this->channels[0]-1500)*15/500.0;
    ch_lock[0].unlock();
    return result;
}

double ppm_pwm_command::Get_Pitch_Command(){
    double result;
    ch_lock[1].lock();
    result=(this->channels[1]-1500)*15/500.0;
    ch_lock[1].unlock();
    return result;
}

double ppm_pwm_command::Get_Yaw(){
    double result;
    ch_lock[3].lock();
    result=(this->channels[3]-1500)*10/500.0;
    ch_lock[3].unlock();
    return result;
}

int ppm_pwm_command::Get_Throttle(){
    double result;
    ch_lock[2].lock();
    result=this->channels[2];
    ch_lock[2].unlock();
    return result;
}

void ppm_pwm_command::Output_To_Motor(int throttle,double pitch,double roll,double yaw){
    if(pitch>0){
        pitch+=0.5;
    }else{
        pitch-=0.5;
    }

    if(roll>0){
        roll+=0.5;
    }else{
        roll-=0.5;
    }

    if(yaw>0){
        yaw+=0.5;
    }else{
        yaw-=0.5;
    }

    int motor1_pulse=throttle+int(pitch+0.5)+int(roll+0.5)-int(yaw+0.5);
    int motor2_pulse=throttle+int(pitch+0.5)-int(roll+0.5)+int(yaw+0.5);
    int motor3_pulse=throttle-int(pitch+0.5)-int(roll+0.5)-int(yaw+0.5);
    int motor4_pulse=throttle-int(pitch+0.5)+int(roll+0.5)+int(yaw+0.5);
    this->motor1->pulsewidth_us(motor1_pulse);
    this->motor2->pulsewidth_us(motor2_pulse);
    this->motor3->pulsewidth_us(motor3_pulse);
    this->motor4->pulsewidth_us(motor4_pulse);
    //printf("motor: %d,%d,%d,%d\n",motor1_pulse,motor2_pulse,motor3_pulse,motor4_pulse);
}

bool ppm_pwm_command::Get_Fly_Allowance(){
    if(this->channels[4]>1500){
        return true;
    }
    return false;
}
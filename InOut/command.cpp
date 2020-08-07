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
    this->motor1->period_us(20000);
    this->motor2->period_us(20000);
    this->motor3->period_us(20000);
    this->motor4->period_us(20000);
    this->motor1->pulsewidth_us(0);
    this->motor2->pulsewidth_us(0);
    this->motor3->pulsewidth_us(0);
    this->motor4->pulsewidth_us(0);
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
    while (true) {
        // 之所以要检测下降沿是要确保之后的变化一定上升沿
        while (ppm_num) {
            ppm_num = this->ppm_in->read();
        }
        while (!ppm_num) {
            ppm_num = this->ppm_in->read();
        }
        // 跳出循环说明收到高电平，此时是上升沿
        this->Store_Channel();
    }
}

void ppm_pwm_command::Store_Channel() {
    int64_t time =
        this->timer_ppm.elapsed_time().count(); // 不能让timer停下来，否则就是0
    this->timer_ppm.reset();
    if (time > 3000) { // 说明刚刚经过一个周期末尾的空白阶段
        if (this->current_channel == PPM_CHANNELS) {
            this->state = true;
        }
        this->current_channel = 0;
    } else if (this->current_channel >= PPM_CHANNELS) {
        printf("system channel numbers are small!\n");
        this->current_channel = 0;
    } else {
        this->channels[this->current_channel] = time;
        this->current_channel = this->current_channel + 1;
    }
}

double ppm_pwm_command::Get_Roll_Command(){//输出滚转角
    return (this->channels[0]-1500)*5/500;
}

double ppm_pwm_command::Get_Pitch_Command(){
    return (this->channels[1]-1500)*5/500;
}

double ppm_pwm_command::Get_Yaw(){
    return (this->channels[3]-1500)*10/500;
}

double ppm_pwm_command::Get_Throttle(){
    return this->channels[2];
}

void ppm_pwm_command::Output_To_Motor(double throttle,double pitch,double roll,double yaw){
    this->motor1->pulsewidth_us(throttle+pitch+roll-yaw);
    this->motor2->pulsewidth_us(throttle+pitch-roll+yaw);
    this->motor3->pulsewidth_us(throttle-pitch-roll-yaw);
    this->motor4->pulsewidth_us(throttle-pitch+roll+yaw);
    printf("%f,%f,%f,%f\n",throttle+pitch+roll-yaw,throttle+pitch-roll+yaw,throttle-pitch-roll-yaw,throttle-pitch+roll+yaw);
}
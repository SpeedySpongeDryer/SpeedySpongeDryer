#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    MotorControl(uint8_t pwm1Pin1, uint8_t pwm1Pin2, uint8_t adcPin,
                 uint8_t pwm2Pin1, uint8_t pwm2Pin2);
    // Constructor to initialize motor control pins / 构造函数，用于初始化电机控制引脚

    void begin();
    // Initialize motor control / 初始化电机控制

    // Simultaneous control methods for both motors
    void startMotorsForward();
    void startMotorsBackward();
    void stopAllMotors();

    void setupADCInterrupt();
    // Setup ADC interrupt for voltage monitoring / 设置 ADC 中断以监控电压

    static bool shouldStop;
    // Static variable to indicate if the motor should stop / 静态变量，用于指示电机是否需要停止

private:
    uint8_t _pwm1Pin1, _pwm1Pin2, _adcPin, _pwm2Pin1, _pwm2Pin2;

    void slowStartBothMotors(uint8_t channel1, uint8_t channel2, int maxDutyCycle);
    // Gradually increase motor speed / 逐步增加电机速度

    static void IRAM_ATTR onTimer();
    // Timer interrupt handler for ADC monitoring / 用于 ADC 监控的定时器中断处理程序

    static MotorControl* instance;
    // Static instance pointer for interrupt handling / 静态实例指针，用于中断处理
};

#endif
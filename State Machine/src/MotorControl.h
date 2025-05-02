#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
    MotorControl(uint8_t pwmPin1, uint8_t pwmPin2, uint8_t adcPin);
    // Constructor to initialize motor control pins / 构造函数，用于初始化电机控制引脚

    void begin();
    // Initialize motor control / 初始化电机控制

    void startMotorForward();
    // Start motor in forward direction / 启动电机正转

    void startMotorBackward();
    // Start motor in backward direction / 启动电机反转

    void stopMotor();
    // Stop the motor / 停止电机

    void setupADCInterrupt();
    // Setup ADC interrupt for voltage monitoring / 设置 ADC 中断以监控电压

    static bool shouldStop;
    // Static variable to indicate if the motor should stop / 静态变量，用于指示电机是否需要停止

private:
    uint8_t _pwmPin1;
    // PWM pin 1 for motor control / 电机控制的 PWM 引脚 1

    uint8_t _pwmPin2;
    // PWM pin 2 for motor control / 电机控制的 PWM 引脚 2

    uint8_t _adcPin;
    // ADC pin for voltage monitoring / 用于电压监控的 ADC 引脚

    void slowStart(uint8_t channel, int maxDutyCycle);
    // Gradually increase motor speed / 逐步增加电机速度

    static void IRAM_ATTR onTimer();
    // Timer interrupt handler for ADC monitoring / 用于 ADC 监控的定时器中断处理程序

    static MotorControl* instance;
    // Static instance pointer for interrupt handling / 静态实例指针，用于中断处理
};

#endif
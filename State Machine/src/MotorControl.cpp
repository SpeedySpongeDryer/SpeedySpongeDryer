#include "MotorControl.h"

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
MotorControl* MotorControl::instance = nullptr;
bool MotorControl::shouldStop = false; // Initialize static variable / 初始化静态变量

MotorControl::MotorControl(uint8_t pwmPin1, uint8_t pwmPin2, uint8_t adcPin)
    : _pwmPin1(pwmPin1), _pwmPin2(pwmPin2), _adcPin(adcPin) {
    instance = this;
}

void MotorControl::begin() {
    pinMode(_pwmPin1, OUTPUT); // Set PWM pin 1 as output / 设置 PWM 引脚 1 为输出
    pinMode(_pwmPin2, OUTPUT); // Set PWM pin 2 as output / 设置 PWM 引脚 2 为输出
    pinMode(_adcPin, INPUT);   // Set ADC pin as input / 设置 ADC 引脚为输入

    ledcSetup(0, 5000, 8); // Channel 0, 5kHz frequency, 8-bit resolution / 通道 0，5kHz 频率，8 位分辨率
    ledcAttachPin(_pwmPin1, 0); // Attach PWM_MOTOR_IN1 to channel 0 / 将 PWM_MOTOR_IN1 连接到通道 0

    ledcSetup(1, 5000, 8); // Channel 1, 5kHz frequency, 8-bit resolution / 通道 1，5kHz 频率，8 位分辨率
    ledcAttachPin(_pwmPin2, 1); // Attach PWM_MOTOR_IN2 to channel 1 / 将 PWM_MOTOR_IN2 连接到通道 1

    ledcWrite(0, 0); // Set duty cycle of channel 0 to 0 / 设置通道 0 的占空比为 0
    ledcWrite(1, 0); // Set duty cycle of channel 1 to 0 / 设置通道 1 的占空比为 0
}

void MotorControl::startMotorForward() {
    digitalWrite(_pwmPin1, HIGH); // Set motor to forward direction / 设置电机为正转方向
    digitalWrite(_pwmPin2, LOW);  // Disable reverse direction / 禁用反转方向
    Serial.println("Motor is rotating forward"); // Print forward rotation message / 打印正转信息
    slowStart(0, 128); // Gradually increase speed / 慢慢加速
}

void MotorControl::startMotorBackward() {
    digitalWrite(_pwmPin1, LOW);  // Disable forward direction / 禁用正转方向
    digitalWrite(_pwmPin2, HIGH); // Set motor to reverse direction / 设置电机为反转方向
    Serial.println("Motor is rotating backward"); // Print backward rotation message / 打印反转信息
    slowStart(1, 95); // Gradually increase speed / 慢慢加速
}

void MotorControl::stopMotor() {
    ledcWrite(0, 0); // Set duty cycle of channel 0 to 0 / 设置通道 0 的占空比为 0
    ledcWrite(1, 0); // Set duty cycle of channel 1 to 0 / 设置通道 1 的占空比为 0
    Serial.println("Motor is stopped"); // Print motor stopped message / 打印电机停止信息
}

void MotorControl::setupADCInterrupt() {
    timer = timerBegin(0, 80, true); // 80 prescaler, 1 MHz count frequency / 80 分频，1 MHz 计数频率
    timerAttachInterrupt(timer, &MotorControl::onTimer, true); // Attach timer interrupt / 附加定时器中断
    timerAlarmWrite(timer, 100000, true); // Trigger interrupt every 100ms / 每 100 毫秒触发一次中断
    timerAlarmEnable(timer); // Enable timer alarm / 启用定时器警报
}

void IRAM_ATTR MotorControl::onTimer() {
    portENTER_CRITICAL_ISR(&timerMux); // Enter critical section / 进入临界区
    int adcValue = analogRead(instance->_adcPin); // Read ADC value / 读取 ADC 值
    float voltage = adcValue * (3.3 / 4095.0); // Convert to voltage / 转换为电压
    if (voltage > 0.40) {
        Serial.println("Error: Voltage exceeds 0.40V"); // Print voltage error / 打印电压错误
        shouldStop = true; // Set flag to stop motor / 设置标志以停止电机
    } else {
        Serial.print("Voltage: "); // Print voltage value / 打印电压值
        Serial.println(voltage, 3); // Print with 3 decimal places / 打印 3 位小数
    }
    portEXIT_CRITICAL_ISR(&timerMux); // Exit critical section / 退出临界区
}

void MotorControl::slowStart(uint8_t channel, int maxDutyCycle) {
    for (int dutyCycle = 0; dutyCycle <= maxDutyCycle; dutyCycle++) {
        ledcWrite(channel, dutyCycle); // Set PWM duty cycle / 设置 PWM 占空比
        delay(23); // Delay 23ms for gradual speed increase / 延迟 23 毫秒以逐步加速
    }
}
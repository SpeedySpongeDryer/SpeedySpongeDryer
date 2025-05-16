#include "MotorControl.h"

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
MotorControl* MotorControl::instance = nullptr;
bool MotorControl::shouldStop = false; // Initialize static variable / 初始化静态变量

MotorControl::MotorControl(uint8_t pwm1Pin1, uint8_t pwm1Pin2, uint8_t adcPin,
                           uint8_t pwm2Pin1, uint8_t pwm2Pin2)
    : _pwm1Pin1(pwm1Pin1), _pwm1Pin2(pwm1Pin2), _adcPin(adcPin),
      _pwm2Pin1(pwm2Pin1), _pwm2Pin2(pwm2Pin2) {
    instance = this;
}

void MotorControl::begin() {
    // Setup first motor pins
    pinMode(_pwm1Pin1, OUTPUT);
    pinMode(_pwm1Pin2, OUTPUT);
    pinMode(_adcPin, INPUT);

    // Setup second motor pins
    pinMode(_pwm2Pin1, OUTPUT);
    pinMode(_pwm2Pin2, OUTPUT);

    // Configure PWM channels for first motor
    // PWM frequency set to 20kHz to be outside audible range
    ledcSetup(0, 20000, 8); // Channel 0, 20kHz frequency, 8-bit resolution
    ledcAttachPin(_pwm1Pin1, 0);
    ledcSetup(1, 20000, 8); // Channel 1, 20kHz frequency, 8-bit resolution
    ledcAttachPin(_pwm1Pin2, 1);

    // Configure PWM channels for second motor
    ledcSetup(2, 20000, 8); // Channel 2, 20kHz frequency, 8-bit resolution
    ledcAttachPin(_pwm2Pin1, 2);
    ledcSetup(3, 20000, 8); // Channel 3, 20kHz frequency, 8-bit resolution
    ledcAttachPin(_pwm2Pin2, 3);

    // Initialize all channels to 0
    stopAllMotors();
}

void MotorControl::startMotorsForward() {
    digitalWrite(_pwm1Pin1, HIGH); // Set motor to forward direction / 设置电机为正转方向
    digitalWrite(_pwm1Pin2, LOW);  // Disable reverse direction / 禁用反转方向
    digitalWrite(_pwm2Pin1, HIGH);
    digitalWrite(_pwm2Pin2, LOW);
    Serial.println("Both motors are rotating forward"); // Print forward rotation message / 打印正转信息
    slowStartBothMotors(0, 2, 200); // Gradually increase speed / 慢慢加速
}

void MotorControl::startMotorsBackward() {
    digitalWrite(_pwm1Pin1, LOW);  // Disable forward direction / 禁用正转方向
    digitalWrite(_pwm1Pin2, HIGH); // Set motor to reverse direction / 设置电机为反转方向
    digitalWrite(_pwm2Pin1, LOW);
    digitalWrite(_pwm2Pin2, HIGH);
    Serial.println("Both motors are rotating backward"); // Print backward rotation message / 打印反转信息
    slowStartBothMotors(1, 3, 200); // Gradually increase speed / 慢慢加速
}

void MotorControl::stopAllMotors() {
    ledcWrite(0, 0); // Set duty cycle of channel 0 to 0 / 设置通道 0 的占空比为 0
    ledcWrite(1, 0); // Set duty cycle of channel 1 to 0 / 设置通道 1 的占空比为 0
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    Serial.println("All motors stopped"); // Print motor stopped message / 打印电机停止信息
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
    // if (voltage > 0.40) {
    //     Serial.println("Error: Voltage exceeds 0.40V"); // Print voltage error / 打印电压错误
    //     shouldStop = true; // Set flag to stop motor / 设置标志以停止电机
    // } else {
        // Serial.print("Voltage: "); // Print voltage value / 打印电压值
        // Serial.println(voltage, 3); // Print with 3 decimal places / 打印 3 位小数
    // }
    portEXIT_CRITICAL_ISR(&timerMux); // Exit critical section / 退出临界区
}

void MotorControl::slowStartBothMotors(uint8_t channel1, uint8_t channel2, int maxDutyCycle) {
    // Gradually increase speed of both motors simultaneously
    for (int dutyCycle = 10; dutyCycle <= maxDutyCycle; dutyCycle+=5) {
        ledcWrite(channel1, dutyCycle);
        ledcWrite(channel2, dutyCycle);
        delay(10); // 10ms delay for gradual acceleration
    }
}
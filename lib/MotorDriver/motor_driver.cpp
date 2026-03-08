/**
 * @file motor_driver.cpp
 * @brief Implementação do controle de motores DC
 */

#include "motor_driver.h"

MotorDriver::MotorDriver() 
    : _leftIA(0), _leftIB(0),
      _rightIA(0), _rightIB(0),
      _leftChannelA(0), _leftChannelB(1),
      _rightChannelA(2), _rightChannelB(3),
      _pwmResolution(8), _maxPWM(255),
      _minSpeed(0), _maxSpeed(255) {
}

void MotorDriver::init(uint8_t leftIA, uint8_t leftIB,
                       uint8_t rightIA, uint8_t rightIB,
                       uint32_t pwmFreq, uint8_t pwmResolution) {
    // Armazena os pinos (L9110S: IA e IB por motor)
    _leftIA = leftIA;
    _leftIB = leftIB;
    _rightIA = rightIA;
    _rightIB = rightIB;
    
    _pwmResolution = pwmResolution;
    _maxPWM = (1 << pwmResolution) - 1;
    
    // Configura os canais PWM LEDC para ESP32 (4 canais para L9110S)
    ledcSetup(_leftChannelA, pwmFreq, pwmResolution);
    ledcSetup(_leftChannelB, pwmFreq, pwmResolution);
    ledcSetup(_rightChannelA, pwmFreq, pwmResolution);
    ledcSetup(_rightChannelB, pwmFreq, pwmResolution);
    
    // Associa os canais aos pinos
    ledcAttachPin(_leftIA, _leftChannelA);
    ledcAttachPin(_leftIB, _leftChannelB);
    ledcAttachPin(_rightIA, _rightChannelA);
    ledcAttachPin(_rightIB, _rightChannelB);
    
    // Inicialmente parado
    brake();
    
    Serial.printf("[MOTOR] L9110S inicializado - PWM: %luHz, %d bits\n", 
                  pwmFreq, pwmResolution);
}

void MotorDriver::setLeftSpeed(int16_t speed) {
    setMotor(_leftChannelA, _leftChannelB, speed);
}

void MotorDriver::setRightSpeed(int16_t speed) {
    setMotor(_rightChannelA, _rightChannelB, speed);
}

void MotorDriver::setSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void MotorDriver::forward(uint8_t speed) {
    setSpeeds(speed, speed);
}

void MotorDriver::backward(uint8_t speed) {
    setSpeeds(-speed, -speed);
}

void MotorDriver::turnLeft(uint8_t speed) {
    setSpeeds(-speed, speed);
}

void MotorDriver::turnRight(uint8_t speed) {
    setSpeeds(speed, -speed);
}

void MotorDriver::curveLeft(uint8_t speed, float turnRatio) {
    turnRatio = constrain(turnRatio, 0.0f, 1.0f);
    int16_t leftSpeed = speed * (1.0f - turnRatio);
    int16_t rightSpeed = speed;
    setSpeeds(leftSpeed, rightSpeed);
}

void MotorDriver::curveRight(uint8_t speed, float turnRatio) {
    turnRatio = constrain(turnRatio, 0.0f, 1.0f);
    int16_t leftSpeed = speed;
    int16_t rightSpeed = speed * (1.0f - turnRatio);
    setSpeeds(leftSpeed, rightSpeed);
}

void MotorDriver::brake() {
    // Freio - todos os canais PWM em 0 (L9110S)
    ledcWrite(_leftChannelA, 0);
    ledcWrite(_leftChannelB, 0);
    ledcWrite(_rightChannelA, 0);
    ledcWrite(_rightChannelB, 0);
}

void MotorDriver::coast() {
    // Roda livre - mesmo comportamento que brake no L9110S
    ledcWrite(_leftChannelA, 0);
    ledcWrite(_leftChannelB, 0);
    ledcWrite(_rightChannelA, 0);
    ledcWrite(_rightChannelB, 0);
}

void MotorDriver::setMinSpeed(uint8_t minSpeed) {
    _minSpeed = minSpeed;
}

void MotorDriver::setMaxSpeed(uint8_t maxSpeed) {
    _maxSpeed = maxSpeed;
}

void MotorDriver::setMotor(uint8_t channelA, uint8_t channelB, int16_t speed) {
    // Limita a velocidade
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    
    // Aplica velocidade mínima (dead-band compensation)
    uint8_t absSpeed = abs(speed);
    if (absSpeed > 0 && absSpeed < _minSpeed) {
        absSpeed = _minSpeed;
    }
    
    // Mapeia para a resolução do PWM
    uint16_t pwmValue = map(absSpeed, 0, 255, 0, _maxPWM);
    
    if (speed > 0) {
        // Para frente: IA = PWM, IB = 0
        ledcWrite(channelA, pwmValue);
        ledcWrite(channelB, 0);
    } else if (speed < 0) {
        // Para trás: IA = 0, IB = PWM
        ledcWrite(channelA, 0);
        ledcWrite(channelB, pwmValue);
    } else {
        // Parado
        ledcWrite(channelA, 0);
        ledcWrite(channelB, 0);
    }
}

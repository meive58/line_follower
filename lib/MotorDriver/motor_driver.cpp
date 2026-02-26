/**
 * @file motor_driver.cpp
 * @brief Implementação do controle de motores DC
 */

#include "motor_driver.h"

MotorDriver::MotorDriver() 
    : _leftEn(0), _leftIn1(0), _leftIn2(0),
      _rightEn(0), _rightIn1(0), _rightIn2(0),
      _leftChannel(0), _rightChannel(1),
      _pwmResolution(8), _maxPWM(255),
      _minSpeed(0), _maxSpeed(255) {
}

void MotorDriver::init(uint8_t leftEn, uint8_t leftIn1, uint8_t leftIn2,
                       uint8_t rightEn, uint8_t rightIn1, uint8_t rightIn2,
                       uint32_t pwmFreq, uint8_t pwmResolution) {
    // Armazena os pinos
    _leftEn = leftEn;
    _leftIn1 = leftIn1;
    _leftIn2 = leftIn2;
    _rightEn = rightEn;
    _rightIn1 = rightIn1;
    _rightIn2 = rightIn2;
    
    _pwmResolution = pwmResolution;
    _maxPWM = (1 << pwmResolution) - 1;
    
    // Configura os pinos de direção como saída
    pinMode(_leftIn1, OUTPUT);
    pinMode(_leftIn2, OUTPUT);
    pinMode(_rightIn1, OUTPUT);
    pinMode(_rightIn2, OUTPUT);
    
    // Configura os canais PWM LEDC para ESP32
    ledcSetup(_leftChannel, pwmFreq, pwmResolution);
    ledcSetup(_rightChannel, pwmFreq, pwmResolution);
    
    // Associa os canais aos pinos
    ledcAttachPin(_leftEn, _leftChannel);
    ledcAttachPin(_rightEn, _rightChannel);
    
    // Inicialmente parado
    brake();
    
    Serial.printf("[MOTOR] Drivers inicializados - PWM: %luHz, %d bits\n", 
                  pwmFreq, pwmResolution);
}

void MotorDriver::setLeftSpeed(int16_t speed) {
    setMotor(_leftChannel, _leftIn1, _leftIn2, speed);
}

void MotorDriver::setRightSpeed(int16_t speed) {
    setMotor(_rightChannel, _rightIn1, _rightIn2, speed);
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
    // Freio - ambos os pinos em HIGH
    digitalWrite(_leftIn1, HIGH);
    digitalWrite(_leftIn2, HIGH);
    digitalWrite(_rightIn1, HIGH);
    digitalWrite(_rightIn2, HIGH);
    
    ledcWrite(_leftChannel, 0);
    ledcWrite(_rightChannel, 0);
}

void MotorDriver::coast() {
    // Roda livre - ambos os pinos em LOW
    digitalWrite(_leftIn1, LOW);
    digitalWrite(_leftIn2, LOW);
    digitalWrite(_rightIn1, LOW);
    digitalWrite(_rightIn2, LOW);
    
    ledcWrite(_leftChannel, 0);
    ledcWrite(_rightChannel, 0);
}

void MotorDriver::setMinSpeed(uint8_t minSpeed) {
    _minSpeed = minSpeed;
}

void MotorDriver::setMaxSpeed(uint8_t maxSpeed) {
    _maxSpeed = maxSpeed;
}

void MotorDriver::setMotor(uint8_t channel, uint8_t in1, uint8_t in2, int16_t speed) {
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
        // Para frente
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    } else if (speed < 0) {
        // Para trás
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    } else {
        // Parado
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
    
    ledcWrite(channel, pwmValue);
}

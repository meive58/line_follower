/**
 * @file pid_controller.cpp
 * @brief Implementação do controlador PID
 */

#include "pid_controller.h"

PIDController::PIDController() 
    : _kp(0), _ki(0), _kd(0),
      _pTerm(0), _iTerm(0), _dTerm(0),
      _lastError(0), _integral(0), _output(0),
      _lastTime(0),
      _minOutput(-255), _maxOutput(255),
      _integralLimit(1000) {
}

void PIDController::init(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    reset();
    
    Serial.printf("[PID] Inicializado - Kp=%.4f, Ki=%.4f, Kd=%.4f\n", _kp, _ki, _kd);
}

void PIDController::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float minOutput, float maxOutput) {
    _minOutput = minOutput;
    _maxOutput = maxOutput;
}

void PIDController::setIntegralLimit(float limit) {
    _integralLimit = limit;
}

float PIDController::compute(float error) {
    // Calcula delta time
    unsigned long now = micros();
    float dt = (now - _lastTime) / 1000000.0f;
    
    // Evita divisão por zero ou primeira execução
    if (_lastTime == 0 || dt <= 0) {
        _lastTime = now;
        _lastError = error;
        return 0;
    }
    
    return compute(error, dt);
}

float PIDController::compute(float error, float dt) {
    // Termo Proporcional
    _pTerm = _kp * error;
    
    // Termo Integral com anti-windup
    _integral += error * dt;
    _integral = constrain(_integral, -_integralLimit, _integralLimit);
    _iTerm = _ki * _integral;
    
    // Termo Derivativo (derivada do erro)
    float derivative = (error - _lastError) / dt;
    _dTerm = _kd * derivative;
    
    // Calcula saída total
    _output = _pTerm + _iTerm + _dTerm;
    
    // Limita a saída
    _output = constrain(_output, _minOutput, _maxOutput);
    
    // Atualiza estado
    _lastError = error;
    _lastTime = micros();
    
    return _output;
}

void PIDController::reset() {
    _integral = 0;
    _lastError = 0;
    _pTerm = 0;
    _iTerm = 0;
    _dTerm = 0;
    _output = 0;
    _lastTime = 0;
}

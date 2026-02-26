/**
 * @file encoder.cpp
 * @brief Implementação do módulo de encoder de quadratura
 */

#include "encoder.h"

// Inicialização das instâncias estáticas
Encoder* Encoder::_instances[MAX_ENCODERS] = {nullptr, nullptr};

Encoder::Encoder() 
    : _pinA(0), _pinB(0), _index(0), _ppr(7), _gearRatio(100),
      _count(0), _lastCount(0), _lastTime(0), _rpm(0) {
}

bool Encoder::init(uint8_t pinA, uint8_t pinB, uint16_t ppr, uint16_t gearRatio, uint8_t index) {
    if (index >= MAX_ENCODERS) {
        Serial.println("[ENCODER] Índice inválido!");
        return false;
    }
    
    _pinA = pinA;
    _pinB = pinB;
    _ppr = ppr;
    _gearRatio = gearRatio;
    _index = index;
    
    // Calcula pulsos totais por revolução da saída (após redução)
    // Multiplicado por 4 para quadratura completa (4 bordas por ciclo)
    _pulsesPerRev = (uint32_t)_ppr * _gearRatio * 4;
    
    // Configura os pinos como entrada com pull-up
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    
    // Registra a instância
    _instances[_index] = this;
    
    // Configura as interrupções
    if (_index == 0) {
        attachInterrupt(digitalPinToInterrupt(_pinA), handleInterrupt0A, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_pinB), handleInterrupt0B, CHANGE);
    } else {
        attachInterrupt(digitalPinToInterrupt(_pinA), handleInterrupt1A, CHANGE);
        attachInterrupt(digitalPinToInterrupt(_pinB), handleInterrupt1B, CHANGE);
    }
    
    _lastTime = micros();
    
    Serial.printf("[ENCODER %d] Inicializado - Pinos A:%d B:%d, %lu pulsos/rev\n", 
                  _index, _pinA, _pinB, _pulsesPerRev);
    
    return true;
}

volatile long Encoder::getCount() {
    return _count;
}

void Encoder::resetCount() {
    noInterrupts();
    _count = 0;
    _lastCount = 0;
    interrupts();
}

float Encoder::getRPM() {
    return _rpm;
}

float Encoder::getRadPerSec() {
    return _rpm * 2.0f * PI / 60.0f;
}

float Encoder::getDistance(float wheelDiameter) {
    float rotations = getRotations();
    return rotations * PI * wheelDiameter; // Distância em mm
}

float Encoder::getRotations() {
    return (float)_count / (float)_pulsesPerRev;
}

void Encoder::update() {
    unsigned long now = micros();
    unsigned long dt = now - _lastTime;
    
    // Calcula RPM a cada 50ms
    if (dt >= 50000) {
        noInterrupts();
        long currentCount = _count;
        interrupts();
        
        long pulses = currentCount - _lastCount;
        
        // RPM = (pulsos / pulsos_por_rev) * (60 / tempo_em_segundos)
        // RPM = (pulsos * 60 * 1000000) / (pulsos_por_rev * dt_em_us)
        _rpm = ((float)pulses * 60000000.0f) / ((float)_pulsesPerRev * (float)dt);
        
        _lastCount = currentCount;
        _lastTime = now;
    }
}

// ISRs para encoder 0
void IRAM_ATTR Encoder::handleInterrupt0A() {
    if (_instances[0] != nullptr) {
        _instances[0]->processInterruptA();
    }
}

void IRAM_ATTR Encoder::handleInterrupt0B() {
    if (_instances[0] != nullptr) {
        _instances[0]->processInterruptB();
    }
}

// ISRs para encoder 1
void IRAM_ATTR Encoder::handleInterrupt1A() {
    if (_instances[1] != nullptr) {
        _instances[1]->processInterruptA();
    }
}

void IRAM_ATTR Encoder::handleInterrupt1B() {
    if (_instances[1] != nullptr) {
        _instances[1]->processInterruptB();
    }
}

// Processamento das interrupções com decodificação de quadratura
void IRAM_ATTR Encoder::processInterruptA() {
    bool a = digitalRead(_pinA);
    bool b = digitalRead(_pinB);
    
    if (a == b) {
        _count++;
    } else {
        _count--;
    }
}

void IRAM_ATTR Encoder::processInterruptB() {
    bool a = digitalRead(_pinA);
    bool b = digitalRead(_pinB);
    
    if (a != b) {
        _count++;
    } else {
        _count--;
    }
}

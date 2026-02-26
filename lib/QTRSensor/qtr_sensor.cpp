/**
 * @file qtr_sensor.cpp
 * @brief Implementação do módulo QTR-8 Digital
 */

#include "qtr_sensor.h"

QTRSensorArray::QTRSensorArray() 
    : _lastPosition(3500), _calibrated(false) {
    memset(_sensorValues, 0, sizeof(_sensorValues));
}

void QTRSensorArray::init(const uint8_t* pins, int8_t emitterPin) {
    _qtr.setTypeRC();
    _qtr.setSensorPins(pins, QTR_SENSOR_COUNT);
    
    if (emitterPin >= 0) {
        _qtr.setEmitterPin(emitterPin);
    }
    
    _calibrated = false;
    Serial.println("[QTR] Sensores inicializados");
}

void QTRSensorArray::calibrate() {
    Serial.println("[QTR] Iniciando calibração...");
    Serial.println("[QTR] Mova o robô sobre a linha por 10 segundos");
    
    // Reset da calibração anterior
    _qtr.calibrate();
    
    for (uint16_t i = 0; i < QTR_CALIBRATION_SAMPLES; i++) {
        _qtr.calibrate();
        
        // Feedback visual a cada 10%
        if (i % (QTR_CALIBRATION_SAMPLES / 10) == 0) {
            Serial.printf("[QTR] Calibração: %d%%\n", (i * 100) / QTR_CALIBRATION_SAMPLES);
        }
        
        delay(20);
    }
    
    _calibrated = true;
    Serial.println("[QTR] Calibração concluída!");
    
    // Mostra os valores de calibração
    Serial.print("[QTR] Min: ");
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
        Serial.printf("%4d ", _qtr.calibrationOn.minimum[i]);
    }
    Serial.println();
    
    Serial.print("[QTR] Max: ");
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
        Serial.printf("%4d ", _qtr.calibrationOn.maximum[i]);
    }
    Serial.println();
}

bool QTRSensorArray::isCalibrated() {
    return _calibrated;
}

void QTRSensorArray::readRaw(uint16_t* values) {
    _qtr.read(values);
}

void QTRSensorArray::readCalibrated(uint16_t* values) {
    _qtr.readCalibrated(values);
}

uint16_t QTRSensorArray::readLinePosition() {
    // Linha preta em fundo branco
    _lastPosition = _qtr.readLineBlack(_sensorValues);
    return _lastPosition;
}

uint16_t QTRSensorArray::readLinePositionWhite() {
    // Linha branca em fundo preto
    _lastPosition = _qtr.readLineWhite(_sensorValues);
    return _lastPosition;
}

bool QTRSensorArray::detectCrossing(uint16_t threshold) {
    _qtr.readCalibrated(_sensorValues);
    
    uint8_t sensorsOnLine = 0;
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
        if (_sensorValues[i] > threshold) {
            sensorsOnLine++;
        }
    }
    
    // Considera cruzamento se mais de 6 sensores detectarem a linha
    return sensorsOnLine >= 6;
}

bool QTRSensorArray::isLineLost(uint16_t threshold) {
    _qtr.readCalibrated(_sensorValues);
    
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
        if (_sensorValues[i] > threshold) {
            return false; // Pelo menos um sensor detecta a linha
        }
    }
    
    return true; // Nenhum sensor detecta a linha
}

int16_t QTRSensorArray::getError() {
    // Centro = 3500 (para 8 sensores: 0-7000)
    // Erro negativo = linha à esquerda
    // Erro positivo = linha à direita
    return (int16_t)_lastPosition - 3500;
}

uint16_t QTRSensorArray::getLastPosition() {
    return _lastPosition;
}

void QTRSensorArray::getSensorValues(uint16_t* values) {
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
        values[i] = _sensorValues[i];
    }
}

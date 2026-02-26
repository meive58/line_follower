/**
 * @file mpu6050.cpp
 * @brief Implementação do módulo MPU-6050
 */

#include "mpu6050.h"

// Sensibilidade padrão (±250°/s para gyro, ±2g para accel)
#define GYRO_SENSITIVITY   131.0f   // LSB/(°/s)
#define ACCEL_SENSITIVITY  16384.0f // LSB/g

MPU6050Sensor::MPU6050Sensor() 
    : _address(0x68), _initialized(false),
      _gyroOffsetX(0), _gyroOffsetY(0), _gyroOffsetZ(0),
      _lastUpdate(0), _filterAlpha(0.98f) {
    _angles = {0, 0, 0};
}

bool MPU6050Sensor::init(uint8_t sdaPin, uint8_t sclPin, uint8_t address) {
    _address = address;
    
    // Inicializa I2C com pinos customizados
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(400000); // 400kHz Fast Mode
    
    // Verifica conexão
    if (!isConnected()) {
        Serial.println("[MPU6050] Sensor não encontrado!");
        return false;
    }
    
    // Acorda o sensor (sai do modo sleep)
    writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00);
    delay(100);
    
    // Configura o sample rate divider (1kHz / (1 + 7) = 125Hz)
    writeRegister(MPU6050_REG_SMPLRT_DIV, 0x07);
    
    // Configura o filtro passa-baixa (DLPF ~44Hz)
    writeRegister(MPU6050_REG_CONFIG, 0x03);
    
    // Configura escala do giroscópio (±250°/s)
    writeRegister(MPU6050_REG_GYRO_CONFIG, 0x00);
    
    // Configura escala do acelerômetro (±2g)
    writeRegister(MPU6050_REG_ACCEL_CONFIG, 0x00);
    
    _initialized = true;
    _lastUpdate = micros();
    
    Serial.println("[MPU6050] Sensor inicializado!");
    return true;
}

bool MPU6050Sensor::isConnected() {
    Wire.beginTransmission(_address);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    
    // Verifica WHO_AM_I (deve retornar 0x68)
    uint8_t whoAmI = readRegister(MPU6050_REG_WHO_AM_I);
    return (whoAmI == 0x68 || whoAmI == 0x72); // 0x72 para alguns clones
}

void MPU6050Sensor::calibrateGyro(uint16_t samples) {
    Serial.println("[MPU6050] Calibrando giroscópio...");
    Serial.println("[MPU6050] Mantenha o sensor IMÓVEL!");
    
    float sumX = 0, sumY = 0, sumZ = 0;
    
    for (uint16_t i = 0; i < samples; i++) {
        int16_t gx = readWord(MPU6050_REG_GYRO_XOUT_H);
        int16_t gy = readWord(MPU6050_REG_GYRO_XOUT_H + 2);
        int16_t gz = readWord(MPU6050_REG_GYRO_XOUT_H + 4);
        
        sumX += gx / GYRO_SENSITIVITY;
        sumY += gy / GYRO_SENSITIVITY;
        sumZ += gz / GYRO_SENSITIVITY;
        
        if (i % (samples / 10) == 0) {
            Serial.printf("[MPU6050] Calibração: %d%%\n", (i * 100) / samples);
        }
        
        delay(2);
    }
    
    _gyroOffsetX = sumX / samples;
    _gyroOffsetY = sumY / samples;
    _gyroOffsetZ = sumZ / samples;
    
    Serial.printf("[MPU6050] Offsets: X=%.2f, Y=%.2f, Z=%.2f\n", 
                  _gyroOffsetX, _gyroOffsetY, _gyroOffsetZ);
    Serial.println("[MPU6050] Calibração concluída!");
}

AccelData MPU6050Sensor::readAccel() {
    AccelData data;
    
    int16_t ax = readWord(MPU6050_REG_ACCEL_XOUT_H);
    int16_t ay = readWord(MPU6050_REG_ACCEL_XOUT_H + 2);
    int16_t az = readWord(MPU6050_REG_ACCEL_XOUT_H + 4);
    
    data.x = ax / ACCEL_SENSITIVITY;
    data.y = ay / ACCEL_SENSITIVITY;
    data.z = az / ACCEL_SENSITIVITY;
    
    return data;
}

GyroData MPU6050Sensor::readGyro() {
    GyroData data;
    
    int16_t gx = readWord(MPU6050_REG_GYRO_XOUT_H);
    int16_t gy = readWord(MPU6050_REG_GYRO_XOUT_H + 2);
    int16_t gz = readWord(MPU6050_REG_GYRO_XOUT_H + 4);
    
    data.x = (gx / GYRO_SENSITIVITY) - _gyroOffsetX;
    data.y = (gy / GYRO_SENSITIVITY) - _gyroOffsetY;
    data.z = (gz / GYRO_SENSITIVITY) - _gyroOffsetZ;
    
    return data;
}

float MPU6050Sensor::readTemperature() {
    int16_t temp = readWord(MPU6050_REG_ACCEL_XOUT_H + 6);
    return (temp / 340.0f) + 36.53f;
}

void MPU6050Sensor::update() {
    // Calcula delta time
    unsigned long now = micros();
    float dt = (now - _lastUpdate) / 1000000.0f;
    _lastUpdate = now;
    
    // Lê sensores
    AccelData accel = readAccel();
    GyroData gyro = readGyro();
    
    // Calcula ângulos do acelerômetro
    float accelRoll = atan2(accel.y, accel.z) * RAD_TO_DEG;
    float accelPitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * RAD_TO_DEG;
    
    // Filtro complementar para roll e pitch
    _angles.roll = _filterAlpha * (_angles.roll + gyro.x * dt) + (1.0f - _filterAlpha) * accelRoll;
    _angles.pitch = _filterAlpha * (_angles.pitch + gyro.y * dt) + (1.0f - _filterAlpha) * accelPitch;
    
    // Integração simples para yaw (não há referência absoluta)
    _angles.yaw += gyro.z * dt;
    
    // Normaliza yaw para -180 a 180
    if (_angles.yaw > 180.0f) _angles.yaw -= 360.0f;
    if (_angles.yaw < -180.0f) _angles.yaw += 360.0f;
}

Angles MPU6050Sensor::getAngles() {
    return _angles;
}

float MPU6050Sensor::getYaw() {
    return _angles.yaw;
}

void MPU6050Sensor::resetYaw() {
    _angles.yaw = 0;
}

float MPU6050Sensor::getAngularVelocityZ() {
    GyroData gyro = readGyro();
    return gyro.z;
}

void MPU6050Sensor::setFilterCoefficient(float alpha) {
    _filterAlpha = constrain(alpha, 0.0f, 1.0f);
}

// Funções auxiliares de comunicação I2C
void MPU6050Sensor::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t MPU6050Sensor::readRegister(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_address, (uint8_t)1);
    return Wire.read();
}

void MPU6050Sensor::readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_address, length);
    
    for (uint8_t i = 0; i < length && Wire.available(); i++) {
        buffer[i] = Wire.read();
    }
}

int16_t MPU6050Sensor::readWord(uint8_t reg) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_address, (uint8_t)2);
    
    int16_t value = Wire.read() << 8;
    value |= Wire.read();
    return value;
}

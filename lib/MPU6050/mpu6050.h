/**
 * @file mpu6050.h
 * @brief Interface para o giroscópio/acelerômetro MPU-6050
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

// Registradores do MPU-6050
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_INT_ENABLE    0x38
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_GYRO_XOUT_H   0x43
#define MPU6050_REG_WHO_AM_I      0x75

// Estrutura para dados do acelerômetro
struct AccelData {
    float x;  // g
    float y;  // g
    float z;  // g
};

// Estrutura para dados do giroscópio
struct GyroData {
    float x;  // graus/segundo
    float y;  // graus/segundo
    float z;  // graus/segundo
};

// Estrutura para ângulos calculados
struct Angles {
    float roll;   // graus
    float pitch;  // graus
    float yaw;    // graus
};

class MPU6050Sensor {
public:
    /**
     * @brief Construtor
     */
    MPU6050Sensor();

    /**
     * @brief Inicializa o sensor
     * @param sdaPin Pino SDA
     * @param sclPin Pino SCL
     * @param address Endereço I2C (0x68 ou 0x69)
     * @return true se inicializado com sucesso
     */
    bool init(uint8_t sdaPin, uint8_t sclPin, uint8_t address = 0x68);

    /**
     * @brief Verifica se o sensor está conectado
     * @return true se conectado
     */
    bool isConnected();

    /**
     * @brief Calibra os offsets do giroscópio
     * @note O sensor deve estar imóvel durante a calibração
     * @param samples Número de amostras para calibração
     */
    void calibrateGyro(uint16_t samples = 1000);

    /**
     * @brief Lê os dados brutos do acelerômetro
     * @return Dados do acelerômetro em g
     */
    AccelData readAccel();

    /**
     * @brief Lê os dados brutos do giroscópio
     * @return Dados do giroscópio em graus/segundo
     */
    GyroData readGyro();

    /**
     * @brief Lê a temperatura do sensor
     * @return Temperatura em graus Celsius
     */
    float readTemperature();

    /**
     * @brief Atualiza os ângulos usando filtro complementar
     * @note Deve ser chamado em loop com intervalo consistente
     */
    void update();

    /**
     * @brief Obtém os ângulos atuais
     * @return Estrutura com roll, pitch e yaw
     */
    Angles getAngles();

    /**
     * @brief Obtém apenas o ângulo yaw (rotação horizontal)
     * @return Ângulo yaw em graus
     */
    float getYaw();

    /**
     * @brief Reseta o ângulo yaw para zero
     */
    void resetYaw();

    /**
     * @brief Obtém a velocidade angular no eixo Z
     * @return Velocidade angular em graus/segundo
     */
    float getAngularVelocityZ();

    /**
     * @brief Define o coeficiente do filtro complementar
     * @param alpha Coeficiente (0.0 a 1.0), padrão 0.98
     */
    void setFilterCoefficient(float alpha);

private:
    uint8_t _address;
    bool _initialized;
    
    // Offsets de calibração
    float _gyroOffsetX;
    float _gyroOffsetY;
    float _gyroOffsetZ;
    
    // Ângulos calculados
    Angles _angles;
    
    // Tempo da última atualização
    unsigned long _lastUpdate;
    
    // Coeficiente do filtro complementar
    float _filterAlpha;
    
    // Funções auxiliares
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void readRegisters(uint8_t reg, uint8_t* buffer, uint8_t length);
    int16_t readWord(uint8_t reg);
};

#endif // MPU6050_H

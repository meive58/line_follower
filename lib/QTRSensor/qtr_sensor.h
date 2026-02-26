/**
 * @file qtr_sensor.h
 * @brief Interface para o array de sensores QTR-8 Digital
 */

#ifndef QTR_SENSOR_H
#define QTR_SENSOR_H

#include <Arduino.h>
#include <QTRSensors.h>

// Número de sensores no array
#define QTR_SENSOR_COUNT 8

// Número de amostras durante a calibração
#define QTR_CALIBRATION_SAMPLES 400

class QTRSensorArray {
public:
    /**
     * @brief Construtor
     */
    QTRSensorArray();

    /**
     * @brief Inicializa os sensores com os pinos configurados
     * @param pins Array com os pinos dos 8 sensores
     * @param emitterPin Pino do LED emissor (-1 se não usar)
     */
    void init(const uint8_t* pins, int8_t emitterPin = -1);

    /**
     * @brief Realiza a calibração dos sensores
     * @note O robô deve ser movido sobre a linha durante a calibração
     */
    void calibrate();

    /**
     * @brief Verifica se os sensores estão calibrados
     * @return true se calibrados
     */
    bool isCalibrated();

    /**
     * @brief Lê os valores brutos dos sensores
     * @param values Array para armazenar os valores (tamanho QTR_SENSOR_COUNT)
     */
    void readRaw(uint16_t* values);

    /**
     * @brief Lê os valores calibrados dos sensores (0-1000)
     * @param values Array para armazenar os valores (tamanho QTR_SENSOR_COUNT)
     */
    void readCalibrated(uint16_t* values);

    /**
     * @brief Calcula a posição da linha
     * @return Posição da linha (0-7000 para 8 sensores)
     *         3500 = linha centralizada
     *         < 3500 = linha à esquerda
     *         > 3500 = linha à direita
     */
    uint16_t readLinePosition();

    /**
     * @brief Calcula a posição da linha (versão para linha branca em fundo preto)
     * @return Posição da linha (0-7000 para 8 sensores)
     */
    uint16_t readLinePositionWhite();

    /**
     * @brief Verifica se todos os sensores estão sobre a linha (cruzamento)
     * @param threshold Limiar para considerar "sobre a linha" (0-1000)
     * @return true se detectou cruzamento
     */
    bool detectCrossing(uint16_t threshold = 500);

    /**
     * @brief Verifica se todos os sensores estão fora da linha (linha perdida)
     * @param threshold Limiar para considerar "fora da linha" (0-1000)
     * @return true se linha perdida
     */
    bool isLineLost(uint16_t threshold = 100);

    /**
     * @brief Obtém o erro em relação ao centro (para PID)
     * @return Erro (-3500 a +3500), 0 = centralizado
     */
    int16_t getError();

    /**
     * @brief Obtém a última posição válida da linha
     * @return Última posição lida
     */
    uint16_t getLastPosition();

    /**
     * @brief Obtém os valores calibrados dos sensores
     * @param values Array para armazenar os valores (tamanho QTR_SENSOR_COUNT)
     */
    void getSensorValues(uint16_t* values);

private:
    QTRSensors _qtr;
    uint16_t _lastPosition;
    bool _calibrated;
    uint16_t _sensorValues[QTR_SENSOR_COUNT];
};

#endif // QTR_SENSOR_H

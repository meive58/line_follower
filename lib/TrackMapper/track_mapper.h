/**
 * @file track_mapper.h
 * @brief Mapeamento da pista e estimativa do raio de curva (encoders + giroscópio)
 */

#ifndef TRACK_MAPPER_H
#define TRACK_MAPPER_H

#include <Arduino.h>
#include "encoder.h"
#include "mpu6050.h"

// Abaixo desta velocidade angular (graus/s) o trecho é considerado reta
#define GYRO_STRAIGHT_THRESHOLD_DPS  3.0f

// Um ponto salvo a cada X mm percorridos
#define TRACK_SAMPLE_DISTANCE_MM     10.0f
#define TRACK_MAX_POINTS             500

struct TrackPoint {
    float distance;  // mm percorridos desde o início
    float yaw;       // graus
    float radius;    // mm (INFINITY = reta)
    float x;         // mm
    float y;         // mm
};

class TrackMapper {
public:
    /**
     * @brief Construtor
     */
    TrackMapper();

    /**
     * @brief Inicializa o mapeador
     * @param left Encoder esquerdo
     * @param right Encoder direito
     * @param mpu Giroscópio
     * @param wheelDiameterMm Diâmetro da roda em mm
     */
    void init(Encoder* left, Encoder* right, MPU6050Sensor* mpu, float wheelDiameterMm);

    /**
     * @brief Limpa o mapa e zera encoders e yaw
     */
    void reset();

    /**
     * @brief Estima o raio da curva atual: R = v / w
     * @return Raio em mm (INFINITY se for reta ou parado)
     */
    float estimateCurveRadius();

    /**
     * @brief Distância média percorrida pelas duas rodas desde o último reset
     * @return Distância em mm
     */
    float getDistanceTraveled();

    /**
     * @brief Atualiza a odometria e salva um ponto a cada TRACK_SAMPLE_DISTANCE_MM
     * @note Deve ser chamado periodicamente enquanto o robô percorre a pista
     */
    void update();

    /**
     * @brief Número de pontos salvos
     */
    uint16_t getPointCount();

    /**
     * @brief Obtém um ponto do mapa
     * @param index Índice do ponto (0 a getPointCount()-1)
     */
    const TrackPoint& getPoint(uint16_t index);

    /**
     * @brief Envia o mapa em CSV
     * @param out Destino (Serial, SerialBT, ...)
     */
    void printMap(Print& out);

private:
    Encoder* _left;
    Encoder* _right;
    MPU6050Sensor* _mpu;
    float _wheelDiameter;

    TrackPoint _points[TRACK_MAX_POINTS];
    uint16_t _count;
    float _x;
    float _y;
    float _lastSavedDistance;
    float _lastDistance;
};

#endif // TRACK_MAPPER_H

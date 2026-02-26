/**
 * @file encoder.h
 * @brief Interface para encoders de quadratura dos motores N20
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// Número máximo de encoders suportados
#define MAX_ENCODERS 2

class Encoder {
public:
    /**
     * @brief Construtor
     */
    Encoder();

    /**
     * @brief Inicializa o encoder
     * @param pinA Pino do canal A
     * @param pinB Pino do canal B
     * @param ppr Pulsos por revolução do encoder
     * @param gearRatio Relação de redução do motor
     * @param index Índice do encoder (0 ou 1)
     * @return true se inicializado com sucesso
     */
    bool init(uint8_t pinA, uint8_t pinB, uint16_t ppr, uint16_t gearRatio, uint8_t index);

    /**
     * @brief Obtém a contagem atual de pulsos
     * @return Número de pulsos (positivo ou negativo)
     */
    volatile long getCount();

    /**
     * @brief Reseta a contagem de pulsos para zero
     */
    void resetCount();

    /**
     * @brief Calcula a velocidade em RPM
     * @return Velocidade em rotações por minuto
     */
    float getRPM();

    /**
     * @brief Calcula a velocidade em rad/s
     * @return Velocidade angular em radianos por segundo
     */
    float getRadPerSec();

    /**
     * @brief Obtém a distância percorrida
     * @param wheelDiameter Diâmetro da roda em mm
     * @return Distância em mm
     */
    float getDistance(float wheelDiameter);

    /**
     * @brief Obtém o número de rotações completas
     * @return Número de rotações
     */
    float getRotations();

    /**
     * @brief Atualiza o cálculo de velocidade
     * @note Deve ser chamado periodicamente (a cada 10-50ms)
     */
    void update();

    // Funções estáticas para as ISRs
    static void IRAM_ATTR handleInterrupt0A();
    static void IRAM_ATTR handleInterrupt0B();
    static void IRAM_ATTR handleInterrupt1A();
    static void IRAM_ATTR handleInterrupt1B();

private:
    uint8_t _pinA;
    uint8_t _pinB;
    uint8_t _index;
    uint16_t _ppr;           // Pulsos por revolução
    uint16_t _gearRatio;     // Relação de redução
    uint32_t _pulsesPerRev;  // Total de pulsos por revolução da saída
    
    volatile long _count;
    long _lastCount;
    unsigned long _lastTime;
    float _rpm;
    
    // Instâncias estáticas para acesso nas ISRs
    static Encoder* _instances[MAX_ENCODERS];
    
    // Função interna de processamento da interrupção
    void processInterruptA();
    void processInterruptB();
};

#endif // ENCODER_H

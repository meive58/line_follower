/**
 * @file motor_driver.h
 * @brief Interface para controle dos motores DC com ponte H
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// Direções do motor
enum MotorDirection {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_BRAKE,
    MOTOR_COAST
};

class MotorDriver {
public:
    /**
     * @brief Construtor
     */
    MotorDriver();

    /**
     * @brief Inicializa os drivers dos motores (L9110S)
     * @param leftIA Pino IA do motor esquerdo (PWM frente)
     * @param leftIB Pino IB do motor esquerdo (PWM trás)
     * @param rightIA Pino IA do motor direito (PWM frente)
     * @param rightIB Pino IB do motor direito (PWM trás)
     * @param pwmFreq Frequência PWM em Hz
     * @param pwmResolution Resolução PWM em bits
     */
    void init(uint8_t leftIA, uint8_t leftIB,
              uint8_t rightIA, uint8_t rightIB,
              uint32_t pwmFreq = 20000, uint8_t pwmResolution = 8);

    /**
     * @brief Define a velocidade do motor esquerdo
     * @param speed Velocidade (-255 a 255)
     */
    void setLeftSpeed(int16_t speed);

    /**
     * @brief Define a velocidade do motor direito
     * @param speed Velocidade (-255 a 255)
     */
    void setRightSpeed(int16_t speed);

    /**
     * @brief Define a velocidade de ambos os motores
     * @param leftSpeed Velocidade do motor esquerdo (-255 a 255)
     * @param rightSpeed Velocidade do motor direito (-255 a 255)
     */
    void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);

    /**
     * @brief Move o robô para frente
     * @param speed Velocidade (0 a 255)
     */
    void forward(uint8_t speed);

    /**
     * @brief Move o robô para trás
     * @param speed Velocidade (0 a 255)
     */
    void backward(uint8_t speed);

    /**
     * @brief Gira o robô para a esquerda (no lugar)
     * @param speed Velocidade (0 a 255)
     */
    void turnLeft(uint8_t speed);

    /**
     * @brief Gira o robô para a direita (no lugar)
     * @param speed Velocidade (0 a 255)
     */
    void turnRight(uint8_t speed);

    /**
     * @brief Curva suave para a esquerda
     * @param speed Velocidade base (0 a 255)
     * @param turnRatio Razão de curva (0.0 a 1.0)
     */
    void curveLeft(uint8_t speed, float turnRatio);

    /**
     * @brief Curva suave para a direita
     * @param speed Velocidade base (0 a 255)
     * @param turnRatio Razão de curva (0.0 a 1.0)
     */
    void curveRight(uint8_t speed, float turnRatio);

    /**
     * @brief Para os motores (freio)
     */
    void brake();

    /**
     * @brief Para os motores (roda livre)
     */
    void coast();

    /**
     * @brief Define a velocidade mínima efetiva
     * @param minSpeed Velocidade mínima (0 a 255)
     */
    void setMinSpeed(uint8_t minSpeed);

    /**
     * @brief Define a velocidade máxima permitida
     * @param maxSpeed Velocidade máxima (0 a 255)
     */
    void setMaxSpeed(uint8_t maxSpeed);

private:
    // Pinos (L9110S: IA e IB por motor)
    uint8_t _leftIA, _leftIB;
    uint8_t _rightIA, _rightIB;
    
    // Canais PWM LEDC (2 por motor para L9110S)
    uint8_t _leftChannelA;   // Canal PWM para IA esquerdo (frente)
    uint8_t _leftChannelB;   // Canal PWM para IB esquerdo (trás)
    uint8_t _rightChannelA;  // Canal PWM para IA direito (frente)
    uint8_t _rightChannelB;  // Canal PWM para IB direito (trás)
    
    // Configurações
    uint8_t _pwmResolution;
    uint16_t _maxPWM;
    uint8_t _minSpeed;
    uint8_t _maxSpeed;
    
    // Funções auxiliares
    void setMotor(uint8_t channelA, uint8_t channelB, int16_t speed);
};

#endif // MOTOR_DRIVER_H

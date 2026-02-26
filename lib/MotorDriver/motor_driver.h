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
     * @brief Inicializa os drivers dos motores
     * @param leftEn Pino PWM do motor esquerdo
     * @param leftIn1 Pino IN1 do motor esquerdo
     * @param leftIn2 Pino IN2 do motor esquerdo
     * @param rightEn Pino PWM do motor direito
     * @param rightIn1 Pino IN1 do motor direito
     * @param rightIn2 Pino IN2 do motor direito
     * @param pwmFreq Frequência PWM em Hz
     * @param pwmResolution Resolução PWM em bits
     */
    void init(uint8_t leftEn, uint8_t leftIn1, uint8_t leftIn2,
              uint8_t rightEn, uint8_t rightIn1, uint8_t rightIn2,
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
    // Pinos
    uint8_t _leftEn, _leftIn1, _leftIn2;
    uint8_t _rightEn, _rightIn1, _rightIn2;
    
    // Canais PWM LEDC
    uint8_t _leftChannel;
    uint8_t _rightChannel;
    
    // Configurações
    uint8_t _pwmResolution;
    uint16_t _maxPWM;
    uint8_t _minSpeed;
    uint8_t _maxSpeed;
    
    // Funções auxiliares
    void setMotor(uint8_t channel, uint8_t in1, uint8_t in2, int16_t speed);
};

#endif // MOTOR_DRIVER_H

/**
 * @file pid_controller.h
 * @brief Controlador PID para seguidor de linha
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    /**
     * @brief Construtor
     */
    PIDController();

    /**
     * @brief Inicializa o controlador com os ganhos
     * @param kp Ganho proporcional
     * @param ki Ganho integral
     * @param kd Ganho derivativo
     */
    void init(float kp, float ki, float kd);

    /**
     * @brief Define os ganhos do PID
     * @param kp Ganho proporcional
     * @param ki Ganho integral
     * @param kd Ganho derivativo
     */
    void setTunings(float kp, float ki, float kd);

    /**
     * @brief Define os limites de saída
     * @param minOutput Saída mínima
     * @param maxOutput Saída máxima
     */
    void setOutputLimits(float minOutput, float maxOutput);

    /**
     * @brief Define o limite do termo integral (anti-windup)
     * @param limit Limite do integral
     */
    void setIntegralLimit(float limit);

    /**
     * @brief Calcula a saída do PID
     * @param error Erro atual (setpoint - medição)
     * @return Saída do controlador
     */
    float compute(float error);

    /**
     * @brief Calcula a saída do PID com delta time manual
     * @param error Erro atual
     * @param dt Delta time em segundos
     * @return Saída do controlador
     */
    float compute(float error, float dt);

    /**
     * @brief Reseta o controlador (integral e derivativo)
     */
    void reset();

    /**
     * @brief Obtém o termo proporcional atual
     */
    float getP() const { return _pTerm; }

    /**
     * @brief Obtém o termo integral atual
     */
    float getI() const { return _iTerm; }

    /**
     * @brief Obtém o termo derivativo atual
     */
    float getD() const { return _dTerm; }

    /**
     * @brief Obtém a última saída calculada
     */
    float getOutput() const { return _output; }

private:
    // Ganhos
    float _kp, _ki, _kd;
    
    // Termos individuais
    float _pTerm, _iTerm, _dTerm;
    
    // Estado interno
    float _lastError;
    float _integral;
    float _output;
    unsigned long _lastTime;
    
    // Limites
    float _minOutput, _maxOutput;
    float _integralLimit;
};

#endif // PID_CONTROLLER_H

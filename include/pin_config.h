/**
 * @file pin_config.h
 * @brief Configuração de pinos do ESP32 para o seguidor de linha
 * 
 * INSTRUÇÕES: Modifique os valores abaixo conforme sua escolha de pinos.
 * Certifique-se de usar pinos válidos do ESP32 e evitar conflitos.
 * 
 * PINOS A EVITAR:
 * - GPIO 0: Boot mode (pode ser usado com cuidado)
 * - GPIO 1: TX0 (usado para debug serial)
 * - GPIO 3: RX0 (usado para debug serial)
 * - GPIO 6-11: Conectados à flash interna (NÃO USAR)
 * - GPIO 34-39: Apenas entrada (sem pull-up interno)
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ============================================================================
// SENSOR QTR-8 DIGITAL (8 sensores infravermelhos)
// ============================================================================
// Conecte os pinos de saída digital do QTR-8A aos pinos abaixo
// Ordem: Sensor mais à ESQUERDA até o mais à DIREITA do robô

#define QTR_SENSOR_0    0   // Sensor 1 (extrema esquerda) - ALTERE CONFORME NECESSÁRIO
#define QTR_SENSOR_1    0   // Sensor 2 - ALTERE CONFORME NECESSÁRIO
#define QTR_SENSOR_2    0   // Sensor 3 - ALTERE CONFORME NECESSÁRIO
#define QTR_SENSOR_3    0   // Sensor 4 - ALTERE CONFORME NECESSÁRIO
#define QTR_SENSOR_4    0   // Sensor 5 - ALTERE CONFORME NECESSÁRIO
#define QTR_SENSOR_5    0   // Sensor 6 - ALTERE CONFORME NECESSÁRIO
#define QTR_SENSOR_6    0   // Sensor 7 - ALTERE CONFORME NECESSÁRIO
#define QTR_SENSOR_7    0   // Sensor 8 (extrema direita) - ALTERE CONFORME NECESSÁRIO

#define QTR_EMITTER_PIN 0   // Pino de controle do LED IR (opcional, -1 se não usar)

// ============================================================================
// GIROSCÓPIO/ACELERÔMETRO MPU-6050
// ============================================================================
// Comunicação I2C - Use os pinos padrão ou configure customizados

#define MPU_SDA_PIN     21  // Pino SDA (padrão ESP32: GPIO 21) - ALTERE CONFORME NECESSÁRIO
#define MPU_SCL_PIN     22  // Pino SCL (padrão ESP32: GPIO 22) - ALTERE CONFORME NECESSÁRIO
#define MPU_INT_PIN     0   // Pino de interrupção (opcional, -1 se não usar) - ALTERE CONFORME NECESSÁRIO

// Endereço I2C do MPU-6050 (0x68 se AD0=LOW, 0x69 se AD0=HIGH)
#define MPU_I2C_ADDRESS 0x68

// ============================================================================
// ENCODERS DOS MOTORES N20
// ============================================================================
// Use pinos com suporte a interrupção para melhor precisão

#define ENCODER_LEFT_A  0   // Canal A do encoder esquerdo - ALTERE CONFORME NECESSÁRIO
#define ENCODER_LEFT_B  0   // Canal B do encoder esquerdo - ALTERE CONFORME NECESSÁRIO

#define ENCODER_RIGHT_A 0   // Canal A do encoder direito - ALTERE CONFORME NECESSÁRIO
#define ENCODER_RIGHT_B 0   // Canal B do encoder direito - ALTERE CONFORME NECESSÁRIO

// Pulsos por revolução do encoder (ajuste conforme seu encoder)
#define ENCODER_PPR     7   // Pulsos por revolução (típico para N20: 7 ou 11)
#define GEAR_RATIO      100 // Relação de redução do motor N20 (ex: 100:1)

// ============================================================================
// CONTROLE DOS MOTORES (Driver Ponte H - L298N, TB6612, DRV8833, etc.)
// ============================================================================

// Motor Esquerdo
#define MOTOR_LEFT_EN   0   // PWM do motor esquerdo (Enable) - ALTERE CONFORME NECESSÁRIO
#define MOTOR_LEFT_IN1  0   // Direção 1 do motor esquerdo - ALTERE CONFORME NECESSÁRIO
#define MOTOR_LEFT_IN2  0   // Direção 2 do motor esquerdo - ALTERE CONFORME NECESSÁRIO

// Motor Direito
#define MOTOR_RIGHT_EN  0   // PWM do motor direito (Enable) - ALTERE CONFORME NECESSÁRIO
#define MOTOR_RIGHT_IN1 0   // Direção 1 do motor direito - ALTERE CONFORME NECESSÁRIO
#define MOTOR_RIGHT_IN2 0   // Direção 2 do motor direito - ALTERE CONFORME NECESSÁRIO

// Configurações PWM
#define PWM_FREQUENCY   20000   // Frequência PWM em Hz (20kHz - fora da faixa audível)
#define PWM_RESOLUTION  8       // Resolução PWM em bits (8 bits = 0-255)
#define PWM_CHANNEL_L   0       // Canal LEDC para motor esquerdo
#define PWM_CHANNEL_R   1       // Canal LEDC para motor direito

// ============================================================================
// BOTÕES E LEDs (Opcional)
// ============================================================================

#define BUTTON_START    0   // Botão de início - ALTERE CONFORME NECESSÁRIO
#define BUTTON_CALIB    0   // Botão de calibração - ALTERE CONFORME NECESSÁRIO
#define LED_STATUS      2   // LED de status (GPIO 2 = LED onboard) - ALTERE CONFORME NECESSÁRIO

// ============================================================================
// BUZZER (Opcional)
// ============================================================================

#define BUZZER_PIN      0   // Pino do buzzer - ALTERE CONFORME NECESSÁRIO

#endif // PIN_CONFIG_H

/**
 * @file main.cpp
 * @brief Código principal do Seguidor de Linha com ESP32
 * 
 * Componentes:
 * - Array de sensores QTR-8 Digital
 * - Giroscópio/Acelerômetro MPU-6050
 * - Encoders de quadratura para motores N20
 * - Motores DC N20 com ponte H
 * 
 * @author [Seu Nome]
 * @date 2026
 */

#include <Arduino.h>
#include "pin_config.h"
#include "qtr_sensor.h"
#include "mpu6050.h"
#include "encoder.h"
#include "motor_driver.h"
#include "pid_controller.h"

// ============================================================================
// CONSTANTES E CONFIGURAÇÕES
// ============================================================================

// Velocidade base do robô (0-255)
#define BASE_SPEED          150

// Ganhos do PID para seguir linha (AJUSTE CONFORME NECESSÁRIO)
#define PID_KP              0.1f
#define PID_KI              0.0001f
#define PID_KD              0.5f

// Intervalo de atualização em ms
#define UPDATE_INTERVAL_MS  5

// Estados do robô
enum RobotState {
    STATE_IDLE,         // Parado, aguardando comando
    STATE_CALIBRATING,  // Calibrando sensores
    STATE_RUNNING,      // Seguindo linha
    STATE_LOST,         // Linha perdida
    STATE_FINISHED      // Fim do percurso
};

// ============================================================================
// OBJETOS GLOBAIS
// ============================================================================

QTRSensorArray qtrSensors;
MPU6050Sensor mpu;
Encoder encoderLeft;
Encoder encoderRight;
MotorDriver motors;
PIDController linePID;

// Estado atual do robô
RobotState currentState = STATE_IDLE;

// Variáveis de controle
unsigned long lastUpdateTime = 0;

// ============================================================================
// PROTÓTIPOS DE FUNÇÕES
// ============================================================================

void initializeSensors();
void calibrateSensors();
void updateSensors();
void followLine();
void handleLineLost();
void printDebugInfo();

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=====================================");
    Serial.println("   SEGUIDOR DE LINHA - ESP32");
    Serial.println("=====================================");
    Serial.println();
    
    // Configura LED de status
    pinMode(LED_STATUS, OUTPUT);
    digitalWrite(LED_STATUS, LOW);
    
    // Configura botões
    pinMode(BUTTON_START, INPUT_PULLUP);
    pinMode(BUTTON_CALIB, INPUT_PULLUP);
    
    // Inicializa todos os sensores e atuadores
    initializeSensors();
    
    Serial.println();
    Serial.println("[SISTEMA] Pronto!");
    Serial.println("[SISTEMA] Pressione CALIB para calibrar");
    Serial.println("[SISTEMA] Pressione START para iniciar");
    Serial.println();
    
    currentState = STATE_IDLE;
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

void loop() {
    // Verifica botão de calibração
    if (digitalRead(BUTTON_CALIB) == LOW && currentState == STATE_IDLE) {
        delay(50); // Debounce
        if (digitalRead(BUTTON_CALIB) == LOW) {
            calibrateSensors();
        }
        while (digitalRead(BUTTON_CALIB) == LOW); // Aguarda soltar
    }
    
    // Verifica botão de início
    if (digitalRead(BUTTON_START) == LOW && currentState == STATE_IDLE) {
        delay(50); // Debounce
        if (digitalRead(BUTTON_START) == LOW) {
            if (qtrSensors.isCalibrated()) {
                Serial.println("[SISTEMA] Iniciando em 3 segundos...");
                for (int i = 3; i > 0; i--) {
                    Serial.printf("[SISTEMA] %d...\n", i);
                    digitalWrite(LED_STATUS, HIGH);
                    delay(500);
                    digitalWrite(LED_STATUS, LOW);
                    delay(500);
                }
                Serial.println("[SISTEMA] GO!");
                currentState = STATE_RUNNING;
                linePID.reset();
            } else {
                Serial.println("[ERRO] Calibre os sensores primeiro!");
            }
        }
        while (digitalRead(BUTTON_START) == LOW); // Aguarda soltar
    }
    
    // Loop de controle com intervalo fixo
    unsigned long now = millis();
    if (now - lastUpdateTime >= UPDATE_INTERVAL_MS) {
        lastUpdateTime = now;
        
        // Atualiza leituras dos sensores
        updateSensors();
        
        // Máquina de estados
        switch (currentState) {
            case STATE_IDLE:
                // LED pisca lentamente
                digitalWrite(LED_STATUS, (millis() / 1000) % 2);
                break;
                
            case STATE_RUNNING:
                // LED aceso
                digitalWrite(LED_STATUS, HIGH);
                followLine();
                break;
                
            case STATE_LOST:
                // LED pisca rapidamente
                digitalWrite(LED_STATUS, (millis() / 100) % 2);
                handleLineLost();
                break;
                
            case STATE_FINISHED:
                motors.brake();
                digitalWrite(LED_STATUS, LOW);
                break;
                
            default:
                break;
        }
    }
}

// ============================================================================
// FUNÇÕES DE INICIALIZAÇÃO
// ============================================================================

void initializeSensors() {
    Serial.println("[INIT] Inicializando sensores...");
    
    // Array de pinos do QTR-8
    uint8_t qtrPins[QTR_SENSOR_COUNT] = {
        QTR_SENSOR_0, QTR_SENSOR_1, QTR_SENSOR_2, QTR_SENSOR_3,
        QTR_SENSOR_4, QTR_SENSOR_5, QTR_SENSOR_6, QTR_SENSOR_7
    };
    
    // Inicializa sensores QTR-8
    qtrSensors.init(qtrPins, QTR_EMITTER_PIN);
    
    // Inicializa MPU-6050
    if (!mpu.init(MPU_SDA_PIN, MPU_SCL_PIN, MPU_I2C_ADDRESS)) {
        Serial.println("[ERRO] Falha ao inicializar MPU-6050!");
    }
    
    // Inicializa encoders
    encoderLeft.init(ENCODER_LEFT_A, ENCODER_LEFT_B, ENCODER_PPR, GEAR_RATIO, 0);
    encoderRight.init(ENCODER_RIGHT_A, ENCODER_RIGHT_B, ENCODER_PPR, GEAR_RATIO, 1);
    
    // Inicializa motores
    motors.init(
        MOTOR_LEFT_EN, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2,
        MOTOR_RIGHT_EN, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2,
        PWM_FREQUENCY, PWM_RESOLUTION
    );
    
    // Inicializa controlador PID
    linePID.init(PID_KP, PID_KI, PID_KD);
    linePID.setOutputLimits(-255, 255);
    
    Serial.println("[INIT] Sensores inicializados!");
}

// ============================================================================
// FUNÇÕES DE CALIBRAÇÃO
// ============================================================================

void calibrateSensors() {
    currentState = STATE_CALIBRATING;
    
    Serial.println("\n========== CALIBRAÇÃO ==========");
    
    // Pisca LED durante calibração
    digitalWrite(LED_STATUS, HIGH);
    
    // Calibra giroscópio (manter parado)
    Serial.println("\n[CALIB] Mantenha o robô IMÓVEL...");
    delay(2000);
    mpu.calibrateGyro(500);
    
    // Calibra sensores de linha (mover sobre a linha)
    Serial.println("\n[CALIB] Mova o robô sobre a linha...");
    delay(1000);
    qtrSensors.calibrate();
    
    digitalWrite(LED_STATUS, LOW);
    
    Serial.println("\n[CALIB] Calibração concluída!");
    Serial.println("================================\n");
    
    currentState = STATE_IDLE;
}

// ============================================================================
// FUNÇÕES DE ATUALIZAÇÃO
// ============================================================================

void updateSensors() {
    // Atualiza MPU-6050 (filtro complementar)
    mpu.update();
    
    // Atualiza cálculo de velocidade dos encoders
    encoderLeft.update();
    encoderRight.update();
}

// ============================================================================
// FUNÇÕES DE CONTROLE
// ============================================================================

void followLine() {
    // Lê posição da linha
    uint16_t position = qtrSensors.readLinePosition();
    
    // Verifica se perdeu a linha
    if (qtrSensors.isLineLost()) {
        currentState = STATE_LOST;
        return;
    }
    
    // Calcula erro (-3500 a +3500)
    int16_t error = qtrSensors.getError();
    
    // Calcula correção PID
    float correction = linePID.compute(error);
    
    // Aplica correção às velocidades dos motores
    int16_t leftSpeed = BASE_SPEED + correction;
    int16_t rightSpeed = BASE_SPEED - correction;
    
    // Garante que as velocidades estejam no range válido
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    // Define velocidades dos motores
    motors.setSpeeds(leftSpeed, rightSpeed);
    
    // Debug (descomente para ver valores)
    // printDebugInfo();
}

void handleLineLost() {
    static unsigned long lostTime = 0;
    static int16_t lastKnownDirection = 0;
    
    // Primeira vez perdendo a linha
    if (lostTime == 0) {
        lostTime = millis();
        lastKnownDirection = qtrSensors.getError();
        Serial.println("[WARN] Linha perdida!");
    }
    
    // Tenta recuperar girando na última direção conhecida
    if (lastKnownDirection < 0) {
        motors.setSpeeds(-80, 80); // Gira para esquerda
    } else {
        motors.setSpeeds(80, -80); // Gira para direita
    }
    
    // Verifica se encontrou a linha
    if (!qtrSensors.isLineLost()) {
        Serial.println("[INFO] Linha recuperada!");
        currentState = STATE_RUNNING;
        lostTime = 0;
        return;
    }
    
    // Timeout - desiste após 3 segundos
    if (millis() - lostTime > 3000) {
        Serial.println("[ERRO] Timeout - parando o robô");
        motors.brake();
        currentState = STATE_IDLE;
        lostTime = 0;
    }
}

// ============================================================================
// FUNÇÕES DE DEBUG
// ============================================================================

void printDebugInfo() {
    static unsigned long lastPrint = 0;
    
    // Imprime a cada 200ms
    if (millis() - lastPrint < 200) return;
    lastPrint = millis();
    
    // Estado dos sensores QTR-8 (visual)
    uint16_t sensorValues[QTR_SENSOR_COUNT];
    qtrSensors.getSensorValues(sensorValues);
    
    Serial.print("QTR:[");
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++) {
        // Threshold de 500 para considerar "sobre a linha"
        if (sensorValues[i] > 500) {
            Serial.print("#");  // Sensor detectando linha (nível alto)
        } else {
            Serial.print("_");  // Sensor fora da linha (nível baixo)
        }
    }
    Serial.print("] | ");
    
    // Posição da linha
    Serial.printf("Pos:%4d | ", qtrSensors.getLastPosition());
    
    // Erro e PID
    Serial.printf("Err:%5d | P:%.1f I:%.1f D:%.1f | ", 
                  qtrSensors.getError(),
                  linePID.getP(), linePID.getI(), linePID.getD());
    
    // Encoders (RPM)
    Serial.printf("L:%.1f R:%.1f RPM | ", 
                  encoderLeft.getRPM(), encoderRight.getRPM());
    
    // Yaw do giroscópio
    Serial.printf("Yaw:%.1f", mpu.getYaw());
    
    Serial.println();
}

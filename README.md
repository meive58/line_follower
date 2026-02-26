# Seguidor de Linha com ESP32

Projeto de robô seguidor de linha utilizando ESP32, sensores QTR-8 Digital, giroscópio MPU-6050 e encoders para motores N20.

## Componentes Utilizados

| Componente | Descrição |
|------------|-----------|
| ESP32 DevKit | Microcontrolador principal |
| QTR-8RC/QTR-8A | Array de 8 sensores infravermelhos para detecção de linha |
| MPU-6050 | Giroscópio e acelerômetro 6-DOF |
| Motor N20 com Encoder | Micro motor DC com encoder de quadratura |
| Ponte H (L298N/TB6612/DRV8833) | Driver para controle dos motores |

## Estrutura do Projeto

```
line_follower/
├── include/
│   └── pin_config.h        # Configuração de pinos (EDITE AQUI!)
├── lib/
│   ├── Encoder/
│   │   ├── encoder.h
│   │   └── encoder.cpp
│   ├── MotorDriver/
│   │   ├── motor_driver.h
│   │   └── motor_driver.cpp
│   ├── MPU6050/
│   │   ├── mpu6050.h
│   │   └── mpu6050.cpp
│   ├── PIDController/
│   │   ├── pid_controller.h
│   │   └── pid_controller.cpp
│   └── QTRSensor/
│       ├── qtr_sensor.h
│       └── qtr_sensor.cpp
├── src/
│   └── main.cpp            # Código principal
├── platformio.ini          # Configuração do PlatformIO
└── README.md               # Este arquivo
```

## Configuração dos Pinos

**IMPORTANTE**: Antes de compilar, edite o arquivo `include/pin_config.h` com os pinos que você escolher.

### Pinos Disponíveis no ESP32

| Pino | Recomendação |
|------|--------------|
| GPIO 0 | Evite (boot mode) |
| GPIO 1 | Evite (TX0 - debug serial) |
| GPIO 2 | LED onboard - pode usar |
| GPIO 3 | Evite (RX0 - debug serial) |
| GPIO 4 | Seguro |
| GPIO 5 | Seguro |
| **GPIO 6-11** | **NÃO USAR (flash interna)** |
| GPIO 12-33 | Seguros para uso geral |
| GPIO 34-39 | Apenas entrada (sem pull-up) |

### Exemplo de Configuração

```c
// Sensor QTR-8 (8 pinos GPIO)
#define QTR_SENSOR_0    32
#define QTR_SENSOR_1    33
#define QTR_SENSOR_2    25
#define QTR_SENSOR_3    26
#define QTR_SENSOR_4    27
#define QTR_SENSOR_5    14
#define QTR_SENSOR_6    12
#define QTR_SENSOR_7    13

// MPU-6050 (I2C)
#define MPU_SDA_PIN     21
#define MPU_SCL_PIN     22

// Encoders (4 pinos com suporte a interrupção)
#define ENCODER_LEFT_A  34
#define ENCODER_LEFT_B  35
#define ENCODER_RIGHT_A 36
#define ENCODER_RIGHT_B 39

// Motores (6 pinos)
#define MOTOR_LEFT_EN   4
#define MOTOR_LEFT_IN1  16
#define MOTOR_LEFT_IN2  17
#define MOTOR_RIGHT_EN  5
#define MOTOR_RIGHT_IN1 18
#define MOTOR_RIGHT_IN2 19

// Botões
#define BUTTON_START    23
#define BUTTON_CALIB    15
```

## Instalação

### Pré-requisitos

1. [Visual Studio Code](https://code.visualstudio.com/)
2. [Extensão PlatformIO](https://platformio.org/install/ide?install=vscode)

### Compilação e Upload

1. Abra a pasta `line_follower` no VS Code
2. Edite `include/pin_config.h` com seus pinos
3. Conecte o ESP32 via USB
4. Clique no botão "Upload" (→) na barra inferior do PlatformIO

## Uso

### Calibração

1. Ligue o robô e aguarde a inicialização
2. Pressione o botão **CALIB**
3. Mantenha o robô **imóvel** durante a calibração do giroscópio (~3s)
4. Mova o robô sobre a linha (preto e branco) por ~10 segundos
5. A calibração será salva automaticamente

### Execução

1. Posicione o robô sobre a linha
2. Pressione o botão **START**
3. Aguarde a contagem regressiva (3, 2, 1...)
4. O robô começará a seguir a linha

### LED de Status

| Estado | LED |
|--------|-----|
| Aguardando | Pisca lento (1Hz) |
| Calibrando | Aceso fixo |
| Seguindo | Aceso fixo |
| Linha perdida | Pisca rápido (10Hz) |

## Ajuste do PID

Os ganhos do PID estão definidos no arquivo `src/main.cpp`:

```c
#define PID_KP  0.1f    // Proporcional
#define PID_KI  0.0001f // Integral
#define PID_KD  0.5f    // Derivativo
```

### Guia de Ajuste

1. **Comece com Ki = 0, Kd = 0**
2. **Aumente Kp** até o robô oscilar levemente
3. **Aumente Kd** para reduzir a oscilação
4. **Adicione Ki** (pequeno) se houver erro persistente

### Valores Sugeridos para Início

| Pista | Kp | Ki | Kd |
|-------|----|----|----| 
| Iniciante (curvas suaves) | 0.05 | 0.0001 | 0.3 |
| Intermediário | 0.1 | 0.0002 | 0.5 |
| Avançado (curvas fechadas) | 0.15 | 0.0005 | 0.8 |

## Diagrama de Conexões

```
                    ┌─────────────────┐
                    │     ESP32       │
                    └─────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│   QTR-8       │   │   MPU-6050    │   │   Ponte H     │
│   (8 pinos)   │   │   (I2C)       │   │   (6 pinos)   │
└───────────────┘   └───────────────┘   └───────────────┘
                                                │
                                        ┌───────┴───────┐
                                        │               │
                                        ▼               ▼
                                ┌───────────┐   ┌───────────┐
                                │ Motor E   │   │ Motor D   │
                                │ + Encoder │   │ + Encoder │
                                └───────────┘   └───────────┘
```

## API dos Módulos

### QTRSensorArray

```cpp
void calibrate();                    // Calibra os sensores
uint16_t readLinePosition();         // Retorna posição (0-7000)
int16_t getError();                  // Retorna erro (-3500 a +3500)
bool isLineLost();                   // Verifica se perdeu a linha
bool detectCrossing();               // Detecta cruzamento
```

### MPU6050Sensor

```cpp
void calibrateGyro(uint16_t samples); // Calibra o giroscópio
void update();                        // Atualiza ângulos (chamar no loop)
float getYaw();                       // Retorna ângulo yaw em graus
void resetYaw();                      // Reseta yaw para zero
```

### Encoder

```cpp
long getCount();                     // Retorna contagem de pulsos
float getRPM();                      // Retorna velocidade em RPM
float getDistance(float diameter);   // Retorna distância percorrida
void resetCount();                   // Reseta contador
```

### MotorDriver

```cpp
void setSpeeds(int16_t left, int16_t right);  // Define velocidades
void forward(uint8_t speed);                   // Move para frente
void brake();                                  // Freia os motores
```

## Solução de Problemas

| Problema | Solução |
|----------|---------|
| ESP32 não conecta | Verifique o driver USB (CP210x ou CH340) |
| Sensores não calibram | Verifique as conexões e alimentação |
| Robô não segue a linha | Recalibre em ambiente adequado (iluminação constante) |
| Robô oscila muito | Reduza Kp ou aumente Kd |
| Robô perde a linha nas curvas | Aumente a velocidade base ou ajuste Kp |
| Motores não funcionam | Verifique alimentação da ponte H |

## Licença

Este projeto é livre para uso educacional.

## Autor

Desenvolvido como projeto de TCC.

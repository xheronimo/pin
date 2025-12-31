// system_config.h - ConfiguraciÃ³n del sistema SCADA
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

// ==============================================
// CONFIGURACIÃ“N DE HARDWARE ESPECÃFICA
// ==============================================

// Pines ESP32-S3 en placa KCK868A16V3
#define PIN_ANALOG_A1     4
#define PIN_ANALOG_A2     6
#define PIN_ANALOG_A3     7
#define PIN_ANALOG_A4     5

// I2C Bus
#define PIN_I2C_SDA       9
#define PIN_I2C_SCL       10

// Direcciones I2C
#define ADDR_RELAY_1      0x24
#define ADDR_RELAY_2      0x25
#define ADDR_INPUT_1      0x21
#define ADDR_INPUT_2      0x22
#define ADDR_EEPROM       0x50
#define ADDR_RTC          0x68
#define ADDR_OLED         0x3C

// 1-Wire
#define PIN_1WIRE_1       47
#define PIN_1WIRE_2       48
#define PIN_1WIRE_3       38

// Ethernet (W5500)
#define PIN_ETH_CLK       42
#define PIN_ETH_MOSI      43
#define PIN_ETH_MISO      44
#define PIN_ETH_CS        15
#define PIN_ETH_INT       2
#define PIN_ETH_RST       1

// RS485
#define PIN_RS485_RX      17
#define PIN_RS485_TX      16
#define PIN_RS485_DE      13  // Driver Enable

// SD Card
#define PIN_SD_MOSI       12
#define PIN_SD_SCK        13
#define PIN_SD_MISO       14
#define PIN_SD_CS         11
#define PIN_SD_CD         21

// RF 433MHz
#define PIN_RF_RX         8
#define PIN_RF_TX         18

// GPIO libres
#define PIN_FREE_1        39
#define PIN_FREE_2        40
#define PIN_FREE_3        41

// ==============================================
// CONFIGURACIÃ“N DEL SISTEMA
// ==============================================

// Modos de operaciÃ³n
typedef enum {
    MODE_MANUAL = 0,
    MODE_AUTO = 1,
    MODE_TEST = 2,
    MODE_MAINTENANCE = 3
} OperationMode;

// Interfaces de red
typedef enum {
    INTERFACE_NONE = 0,
    INTERFACE_ETHERNET = 1,
    INTERFACE_WIFI = 2,
    INTERFACE_4G = 3
} NetworkInterface;

// Prioridad de interfaces (para fallback)
#define INTERFACE_PRIORITY INTERFACE_ETHERNET
#define INTERFACE_FALLBACK_1 INTERFACE_WIFI
#define INTERFACE_FALLBACK_2 INTERFACE_4G

// Sensores
typedef enum {
    SENSOR_TEMPERATURE = 0,
    SENSOR_PRESSURE = 1,
    SENSOR_LEVEL = 2,
    SENSOR_FLOW = 3,
    SENSOR_VOLTAGE = 4,
    SENSOR_CURRENT = 5,
    SENSOR_POWER = 6,
    SENSOR_FREQUENCY = 7,
    SENSOR_ANALOG = 8,
    SENSOR_DIGITAL = 9
} SensorType;

// Alarmas
typedef enum {
    ALARM_NONE = 0,
    ALARM_LOW = 1,
    ALARM_MEDIUM = 2,
    ALARM_HIGH = 3,
    ALARM_CRITICAL = 4
} AlarmPriority;

// Estados del motor
typedef enum {
    MOTOR_OFF = 0,
    MOTOR_STARTING = 1,
    MOTOR_RUNNING = 2,
    MOTOR_STOPPING = 3,
    MOTOR_FAULT = 4
} MotorState;

// ==============================================
// ESTRUCTURAS DE DATOS
// ==============================================

// ConfiguraciÃ³n del sistema
typedef struct {
    char deviceName[32];
    char deviceLocation[64];
    OperationMode operationMode;
    
    // ConfiguraciÃ³n de motores
    bool motorAlternationEnabled;
    uint32_t motorAlternationTime;  // en segundos
    float motorSpeedReference;      // 0-100%
    
    // Umbrales de nivel de agua
    float lowLevelThreshold;
    float highLevelThreshold;
    float midLevelThreshold;
    
    // ConfiguraciÃ³n de red
    uint32_t serverUpdateInterval;  // ms
    uint32_t dataLogInterval;       // ms
    
    // Alarmas
    float pressureAlarmHigh;
    float pressureAlarmLow;
    float levelAlarmHigh;
    float levelAlarmLow;
    float temperatureAlarmHigh;
    
    // ConfiguraciÃ³n de comunicaciÃ³n
    uint32_t modbusTimeout;
    uint32_t modbusRetries;
    uint32_t rs485BaudRate;
} SystemConfig;

// Estado del sistema
typedef struct {
    uint32_t startTime;
    uint32_t uptime;
    OperationMode operationMode;
    uint32_t freeHeap;
    uint32_t minFreeHeap;
    float temperature;
    uint32_t lastServerUpdate;
    uint32_t lastDataLog;
    uint8_t networkStatus;  // bitmask de interfaces disponibles
} SystemStatus;

// Datos del sensor
typedef struct {
    SensorType type;
    uint8_t id;
    float value;
    uint32_t timestamp;
    uint8_t quality;  // 0-100%
    bool alarm;
} SensorData;

// Evento de alarma
typedef struct {
    AlarmPriority priority;
    char source[32];
    char message[128];
    uint32_t timestamp;
    char phoneNumbers[5][16];  // NÃºmeros para SMS
    uint8_t smsCount;
} AlarmEvent;

// Comando de motor
typedef struct {
    uint8_t motorId;
    MotorState command;
    float speed;  // 0-100%
    bool immediate;
} MotorCommand;

// Paquete de red
typedef struct {
    NetworkInterface interface;
    String data;
    uint8_t priority;
    uint8_t retryCount;
} NetworkPacket;

// ActualizaciÃ³n de display
typedef struct {
    char line1[21];
    char line2[21];
    char line3[21];
    char line4[21];
    uint8_t updateType;  // 0=completo, 1=parcial
} DisplayUpdate;

// Estado del motor
typedef struct {
    MotorState state;
    float currentSpeed;
    float currentPower;
    uint32_t runningHours;
    uint32_t startCount;
    uint8_t faults;
    bool autoMode;
} MotorStatus;

// ==============================================
// CONSTANTES
// ==============================================

// TamaÃ±os de cola
#define QUEUE_SENSOR_SIZE      20
#define QUEUE_ALARM_SIZE       30
#define QUEUE_MOTOR_SIZE       10
#define QUEUE_NETWORK_SIZE     20
#define QUEUE_DISPLAY_SIZE     10

// Prioridades de tareas
#define TASK_PRIORITY_SYSTEM_MONITOR   3
#define TASK_PRIORITY_SENSOR_READER    4
#define TASK_PRIORITY_MOTOR_CONTROL    4
#define TASK_PRIORITY_ALARM_PROCESSOR  5  // MÃ¡xima prioridad
#define TASK_PRIORITY_NETWORK_MANAGER  2
#define TASK_PRIORITY_WEB_SERVER       1
#define TASK_PRIORITY_DATA_LOGGER      2
#define TASK_PRIORITY_DISPLAY_MANAGER  1

// Stack sizes
#define STACK_SYSTEM_MONITOR   4096
#define STACK_SENSOR_READER    8192
#define STACK_MOTOR_CONTROL    4096
#define STACK_ALARM_PROCESSOR  4096
#define STACK_NETWORK_MANAGER  8192
#define STACK_WEB_SERVER       16384
#define STACK_DATA_LOGGER      4096
#define STACK_DISPLAY_MANAGER  2048

// ==============================================
// VARIABLES GLOBALES (extern)
// ==============================================

extern SystemConfig systemConfig;
extern SystemStatus systemStatus;
extern SemaphoreHandle_t xI2CMutex;
extern SemaphoreHandle_t xSPIMutex;
extern SemaphoreHandle_t xRS485Mutex;
extern SemaphoreHandle_t xDataMutex;
extern SemaphoreHandle_t xConfigMutex;

// ==============================================
// PROTOTIPOS DE FUNCIONES
// ==============================================

void loadSystemConfig();
void saveSystemConfig();
void initHardware();
void performSystemMaintenance();

// Funciones de sensor
void readAnalogSensors();
void readMicrowaveLevelSensor();
void readPowerMonitor();
void readMultiplexedAnalog();
float getWaterLevel();

// Funciones de motor
void startMotor(uint8_t motorId, uint8_t mode);
void stopMotor(uint8_t motorId);
void controlMotorSpeed(MotorStatus motors[2]);
void automaticMotorAlternation(MotorStatus motors[2]);
void processMotorCommand(MotorCommand cmd, MotorStatus motors[2]);

// Funciones de red
bool checkEthernetConnection();
bool checkWiFiConnection();
bool check4GConnection();
bool sendNetworkPacket(NetworkPacket packet, NetworkInterface interface);
NetworkInterface getFallbackInterface(NetworkInterface current);
void sendDataToServer(NetworkInterface interface);
void sendSMS(const char* message, const char* numbers[5], uint8_t count);
void sendTelegram(const char* message);
void addToAlarmQueue(AlarmEvent alarm);

// Funciones de alarma
void processAlarmEvent(AlarmEvent &alarm);
void logAlarm(AlarmEvent &alarm);

// Funciones de utilidad
String generateSystemJSON();
float convertToProcessValue(uint16_t raw, uint8_t channel);
void selectAnalogChannel(uint8_t channel);
uint8_t getAnalogPin(uint8_t channel);

// Funciones de display
void updateOLEDDisplay(DisplayUpdate update);

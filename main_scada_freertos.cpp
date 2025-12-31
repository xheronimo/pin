// main_scada_freertos.cpp - Sistema SCADA con FreeRTOS para ESP32-S3
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>

// ConfiguraciÃ³n de hardware especÃ­fica para KCK868A16V3
#define HARDWARE_KCK868A16V3

// Componentes del sistema
#include "config.h"
#include "system_manager.h"
#include "task_manager.h"
#include "hardware_manager.h"
#include "motor_controller.h"
#include "sensor_manager.h"
#include "network_manager.h"
#include "alarm_manager.h"
#include "data_logger.h"
#include "web_server.h"
#include "multiplex_analog.h"
#include "power_monitor.h"

// Mutex globales para protecciÃ³n de recursos compartidos
SemaphoreHandle_t xI2CMutex = NULL;
SemaphoreHandle_t xSPIMutex = NULL;
SemaphoreHandle_t xRS485Mutex = NULL;
SemaphoreHandle_t xDataMutex = NULL;
SemaphoreHandle_t xConfigMutex = NULL;

// Colas para comunicaciÃ³n entre tareas
QueueHandle_t xSensorQueue = NULL;
QueueHandle_t xAlarmQueue = NULL;
QueueHandle_t xMotorQueue = NULL;
QueueHandle_t xNetworkQueue = NULL;
QueueHandle_t xDisplayQueue = NULL;

// Variables del sistema
SystemStatus systemStatus = {0};
SystemConfig systemConfig;
bool systemInitialized = false;

// Prototipos de tareas
void vTaskSystemMonitor(void *pvParameters);
void vTaskSensorReader(void *pvParameters);
void vTaskMotorControl(void *pvParameters);
void vTaskNetworkManager(void *pvParameters);
void vTaskAlarmProcessor(void *pvParameters);
void vTaskWebServer(void *pvParameters);
void vTaskDataLogger(void *pvParameters);
void vTaskDisplayManager(void *pvParameters);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n========================================");
    Serial.println("  SISTEMA SCADA INDUSTRIAL - FreeRTOS");
    Serial.println("  Hardware: KCK868A16V3");
    Serial.println("========================================\n");
    
    // Inicializar mutex
    xI2CMutex = xSemaphoreCreateMutex();
    xSPIMutex = xSemaphoreCreateMutex();
    xRS485Mutex = xSemaphoreCreateMutex();
    xDataMutex = xSemaphoreCreateMutex();
    xConfigMutex = xSemaphoreCreateMutex();
    
    // Inicializar colas
    xSensorQueue = xQueueCreate(20, sizeof(SensorData));
    xAlarmQueue = xQueueCreate(30, sizeof(AlarmEvent));
    xMotorQueue = xQueueCreate(10, sizeof(MotorCommand));
    xNetworkQueue = xQueueCreate(20, sizeof(NetworkPacket));
    xDisplayQueue = xQueueCreate(10, sizeof(DisplayUpdate));
    
    // Cargar configuraciÃ³n
    if (xSemaphoreTake(xConfigMutex, portMAX_DELAY)) {
        loadSystemConfig();
        xSemaphoreGive(xConfigMutex);
    }
    
    // Inicializar hardware
    initHardware();
    
    // Crear tareas FreeRTOS
    xTaskCreatePinnedToCore(
        vTaskSystemMonitor,    // FunciÃ³n de la tarea
        "SystemMonitor",       // Nombre
        4096,                  // Stack size
        NULL,                  // ParÃ¡metros
        3,                     // Prioridad (alta)
        NULL,                  // Handle
        0                      // Core 0
    );
    
    xTaskCreatePinnedToCore(
        vTaskSensorReader,
        "SensorReader",
        8192,                  // MÃ¡s stack para Modbus/RS485
        NULL,
        4,                     // Alta prioridad para sensores
        NULL,
        1                      // Core 1
    );
    
    xTaskCreatePinnedToCore(
        vTaskMotorControl,
        "MotorControl",
        4096,
        NULL,
        4,                     // Alta prioridad para motores
        NULL,
        1
    );
    
    xTaskCreatePinnedToCore(
        vTaskNetworkManager,
        "NetworkManager",
        8192,                  // Stack grande para red
        NULL,
        2,                     // Prioridad media
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        vTaskAlarmProcessor,
        "AlarmProcessor",
        4096,
        NULL,
        5,                     // MÃ¡xima prioridad para alarmas
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        vTaskWebServer,
        "WebServer",
        16384,                 // Stack grande para servidor web
        NULL,
        1,                     // Prioridad baja
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        vTaskDataLogger,
        "DataLogger",
        4096,
        NULL,
        2,
        NULL,
        1
    );
    
    xTaskCreatePinnedToCore(
        vTaskDisplayManager,
        "DisplayManager",
        2048,
        NULL,
        1,
        NULL,
        0
    );
    
    systemInitialized = true;
    systemStatus.startTime = millis();
    systemStatus.operationMode = MODE_AUTO;
    
    Serial.println("\nâœ… Sistema SCADA inicializado con FreeRTOS");
    Serial.println("   Tareas en ejecuciÃ³n...\n");
}

void loop() {
    // En FreeRTOS, loop() puede estar vacÃ­o o manejar tareas de baja prioridad
    vTaskDelay(pdMS_TO_TICKS(1000));  // Delay de 1 segundo
    
    // Tarea de mantenimiento del sistema (opcional en loop)
    static uint32_t lastMaintenance = 0;
    if (millis() - lastMaintenance > 60000) {  // Cada minuto
        performSystemMaintenance();
        lastMaintenance = millis();
    }
}

// Tarea: Monitor del sistema
void vTaskSystemMonitor(void *pvParameters) {
    Serial.println("ðŸ“Š Tarea SystemMonitor iniciada");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5000);  // 5 segundos
    
    while (1) {
        // Monitorear estado del sistema
        systemStatus.freeHeap = esp_get_free_heap_size();
        systemStatus.minFreeHeap = esp_get_minimum_free_heap_size();
        systemStatus.uptime = (millis() - systemStatus.startTime) / 1000;
        
        // Verificar temperatura del ESP
        systemStatus.temperature = temperatureRead();  // FunciÃ³n interna ESP
        
        // Verificar tareas
        UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        
        // Log cada 30 segundos
        static uint32_t lastLog = 0;
        if (millis() - lastLog > 30000) {
            Serial.printf("ðŸ”§ Estado Sistema: Heap=%d, Temp=%.1fÂ°C, Uptime=%d s\n",
                         systemStatus.freeHeap, systemStatus.temperature, systemStatus.uptime);
            lastLog = millis();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// Tarea: Lectura de sensores
void vTaskSensorReader(void *pvParameters) {
    Serial.println("ðŸ“¡ Tarea SensorReader iniciada");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // Temporizadores para cada tipo de sensor
    uint32_t lastAnalogRead = 0;
    uint32_t lastModbusRead = 0;
    uint32_t lastMicrowaveRead = 0;
    uint32_t lastPowerRead = 0;
    
    while (1) {
        uint32_t now = millis();
        
        // 1. Lectura de entradas analÃ³gicas (4-20mA) - cada 500ms
        if (now - lastAnalogRead >= 500) {
            readAnalogSensors();
            lastAnalogRead = now;
        }
        
        // 2. Lectura Modbus/RS485 (sensor microondas nivel) - cada 1s
        if (now - lastModbusRead >= 1000) {
            readMicrowaveLevelSensor();
            lastModbusRead = now;
        }
        
        // 3. Lectura medidor de potencia trifÃ¡sico - cada 2s
        if (now - lastPowerRead >= 2000) {
            readPowerMonitor();
            lastPowerRead = now;
        }
        
        // 4. Lectura multiplexor 16 canales - cada 100ms
        readMultiplexedAnalog();
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Delay de 10ms
    }
}

// Tarea: Control de motores
void vTaskMotorControl(void *pvParameters) {
    Serial.println("âš™ï¸ Tarea MotorControl iniciada");
    
    MotorState motorStates[2] = {0};  // Dos motores
    MotorCommand motorCmd;
    
    while (1) {
        // Verificar comandos en cola
        if (xQueueReceive(xMotorQueue, &motorCmd, 0) == pdTRUE) {
            processMotorCommand(motorCmd, motorStates);
        }
        
        // Control automÃ¡tico de alternancia
        if (systemStatus.operationMode == MODE_AUTO) {
            automaticMotorAlternation(motorStates);
        }
        
        // Control con variador de velocidad
        if (systemConfig.motorSpeedControl) {
            controlMotorSpeed(motorStates);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // 20Hz para control de motores
    }
}

// Tarea: GestiÃ³n de red y comunicaciones
void vTaskNetworkManager(void *pvParameters) {
    Serial.println("ðŸŒ Tarea NetworkManager iniciada");
    
    NetworkPacket packet;
    NetworkInterface activeInterface = INTERFACE_NONE;
    
    while (1) {
        // Prioridad de interfaces: Ethernet > WiFi > 4G
        if (checkEthernetConnection()) {
            activeInterface = INTERFACE_ETHERNET;
        } else if (checkWiFiConnection()) {
            activeInterface = INTERFACE_WIFI;
        } else if (check4GConnection()) {
            activeInterface = INTERFACE_4G;
        } else {
            activeInterface = INTERFACE_NONE;
        }
        
        // Procesar paquetes en cola
        if (xQueueReceive(xNetworkQueue, &packet, 0) == pdTRUE) {
            sendNetworkPacket(packet, activeInterface);
        }
        
        // EnvÃ­o periÃ³dico de datos al servidor
        static uint32_t lastServerUpdate = 0;
        if (now - lastServerUpdate >= systemConfig.serverUpdateInterval) {
            sendDataToServer(activeInterface);
            lastServerUpdate = now;
        }
        
        // Verificar SMS pendientes
        checkPendingSMS();
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Tarea: Procesamiento de alarmas
void vTaskAlarmProcessor(void *pvParameters) {
    Serial.println("ðŸš¨ Tarea AlarmProcessor iniciada");
    
    AlarmEvent alarm;
    
    while (1) {
        // Procesar alarmas en cola (alta prioridad)
        if (xQueueReceive(xAlarmQueue, &alarm, portMAX_DELAY) == pdTRUE) {
            processAlarmEvent(alarm);
        }
    }
}

void initHardware() {
    Serial.println("ðŸ”§ Inicializando hardware...");
    
    // 1. I2C para expanders y perifÃ©ricos
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);  // Fast mode
    
    // 2. Inicializar PCF8574 para multiplexaciÃ³n
    initPCF8574Expanders();
    
    // 3. Inicializar Ethernet (W5500)
    initEthernet();
    
    // 4. Inicializar RS485
    initRS485();
    
    // 5. Inicializar SD Card
    initSDCard();
    
    // 6. Inicializar GSM 4G
    initGSMModule();
    
    // 7. Inicializar RTC
    initRTC();
    
    // 8. Inicializar display OLED
    initOLED();
    
    Serial.println("âœ… Hardware inicializado");
}

void processAlarmEvent(AlarmEvent &alarm) {
    // Prioridad de envÃ­o segÃºn tipo de alarma
    switch(alarm.priority) {
        case ALARM_CRITICAL:
            // Enviar por todos los medios inmediatamente
            sendSMS(alarm.message, alarm.phoneNumbers, alarm.smsCount);
            sendTelegram(alarm.message);
            sendToServer(alarm.message, INTERFACE_PRIORITY);
            break;
            
        case ALARM_HIGH:
            // Enviar por SMS y servidor
            sendSMS(alarm.message, alarm.phoneNumbers, alarm.smsCount);
            sendToServer(alarm.message, INTERFACE_PRIORITY);
            break;
            
        case ALARM_MEDIUM:
            // Enviar solo al servidor
            sendToServer(alarm.message, INTERFACE_AVAILABLE);
            break;
            
        case ALARM_LOW:
            // Enviar en prÃ³ximo ciclo
            addToAlarmQueue(alarm);
            break;
    }
    
    // Registrar en log
    logAlarm(alarm);
}

void readMultiplexedAnalog() {
    static uint8_t currentChannel = 0;
    
    // Seleccionar canal en el multiplexor
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10))) {
        // Usar PCF8574 para controlar multiplexor
        selectAnalogChannel(currentChannel);
        
        // Leer valor del ADC
        uint16_t rawValue = analogRead(getAnalogPin(currentChannel));
        
        // Convertir a valor de proceso
        float processValue = convertToProcessValue(rawValue, currentChannel);
        
        // Actualizar estructura de datos
        SensorData sensorData;
        sensorData.type = SENSOR_ANALOG;
        sensorData.channel = currentChannel;
        sensorData.value = processValue;
        sensorData.timestamp = millis();
        
        // Enviar a cola
        xQueueSend(xSensorQueue, &sensorData, 0);
        
        // Siguiente canal
        currentChannel = (currentChannel + 1) % 16;
        
        xSemaphoreGive(xI2CMutex);
    }
}

void automaticMotorAlternation(MotorState motors[2]) {
    // LÃ³gica de alternancia automÃ¡tica segÃºn nivel del depÃ³sito
    float waterLevel = getWaterLevel();
    
    if (waterLevel < systemConfig.lowLevelThreshold) {
        // Arrancar motor principal
        if (!motors[0].running) {
            startMotor(0, MOTOR_PRIMARY);
        }
    } else if (waterLevel > systemConfig.highLevelThreshold) {
        // Detener todos los motores
        stopMotor(0);
        stopMotor(1);
    } else if (waterLevel > systemConfig.midLevelThreshold) {
        // Alternar motores basado en horas de operaciÃ³n
        if (motors[0].runningHours > motors[1].runningHours) {
            stopMotor(0);
            startMotor(1, MOTOR_SECONDARY);
        } else {
            stopMotor(1);
            startMotor(0, MOTOR_PRIMARY);
        }
    }
}

void sendDataToServer(NetworkInterface interface) {
    // Crear paquete JSON con datos del sistema
    String jsonData = generateSystemJSON();
    
    NetworkPacket packet;
    packet.data = jsonData;
    packet.priority = DATA_PRIORITY_HIGH;
    packet.interface = interface;
    
    // Intentar enviar con reintentos
    for (int retry = 0; retry < 3; retry++) {
        if (sendNetworkPacket(packet, interface)) {
            // EnvÃ­o exitoso
            systemStatus.lastServerUpdate = millis();
            break;
        } else {
            // FallÃ³, intentar con otro interface
            interface = getFallbackInterface(interface);
            vTaskDelay(pdMS_TO_TICKS(1000 * (retry + 1)));  // Backoff exponencial
        }
    }
}

// task_manager.cpp - Gestor de tareas FreeRTOS
#include "system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// Handles de tareas
static TaskHandle_t xSystemMonitorHandle = NULL;
static TaskHandle_t xSensorReaderHandle = NULL;
static TaskHandle_t xMotorControlHandle = NULL;
static TaskHandle_t xNetworkManagerHandle = NULL;
static TaskHandle_t xAlarmProcessorHandle = NULL;
static TaskHandle_t xWebServerHandle = NULL;
static TaskHandle_t xDataLoggerHandle = NULL;
static TaskHandle_t xDisplayManagerHandle = NULL;

// Timers para tareas periÃ³dicas
static TimerHandle_t xSensorTimer = NULL;
static TimerHandle_t xMotorTimer = NULL;
static TimerHandle_t xNetworkTimer = NULL;

void initTaskManager() {
    Serial.println("ðŸ”„ Inicializando gestor de tareas...");
    
    // Crear timers
    xSensorTimer = xTimerCreate(
        "SensorTimer",
        pdMS_TO_TICKS(100),  // 100ms
        pdTRUE,              // Auto-reload
        (void*)0,            // ID
        sensorTimerCallback
    );
    
    xMotorTimer = xTimerCreate(
        "MotorTimer",
        pdMS_TO_TICKS(50),   // 50ms (20Hz)
        pdTRUE,
        (void*)1,
        motorTimerCallback
    );
    
    xNetworkTimer = xTimerCreate(
        "NetworkTimer",
        pdMS_TO_TICKS(100),  // 100ms
        pdTRUE,
        (void*)2,
        networkTimerCallback
    );
    
    // Iniciar timers
    if (xSensorTimer != NULL) xTimerStart(xSensorTimer, 0);
    if (xMotorTimer != NULL) xTimerStart(xMotorTimer, 0);
    if (xNetworkTimer != NULL) xTimerStart(xNetworkTimer, 0);
    
    Serial.println("âœ… Gestor de tareas inicializado");
}

// Callbacks de timers
void sensorTimerCallback(TimerHandle_t xTimer) {
    // SeÃ±alizar a la tarea de lectura de sensores
    if (xSensorReaderHandle != NULL) {
        xTaskNotify(xSensorReaderHandle, 0x01, eSetBits);
    }
}

void motorTimerCallback(TimerHandle_t xTimer) {
    // SeÃ±alizar a la tarea de control de motores
    if (xMotorControlHandle != NULL) {
        xTaskNotify(xMotorControlHandle, 0x01, eSetBits);
    }
}

void networkTimerCallback(TimerHandle_t xTimer) {
    // SeÃ±alizar a la tarea de red
    if (xNetworkManagerHandle != NULL) {
        xTaskNotify(xNetworkManagerHandle, 0x01, eSetBits);
    }
}

// GestiÃ³n de prioridades dinÃ¡micas
void adjustTaskPriorities() {
    static uint32_t lastAdjustment = 0;
    uint32_t now = millis();
    
    if (now - lastAdjustment > 10000) {  // Cada 10 segundos
        // Ajustar prioridades basado en estado del sistema
        UBaseType_t systemPriority = TASK_PRIORITY_SYSTEM_MONITOR;
        
        // Si hay alarmas crÃ­ticas, subir prioridad del procesador de alarmas
        if (uxQueueMessagesWaiting(xAlarmQueue) > 5) {
            vTaskPrioritySet(xAlarmProcessorHandle, TASK_PRIORITY_ALARM_PROCESSOR + 1);
        } else {
            vTaskPrioritySet(xAlarmProcessorHandle, TASK_PRIORITY_ALARM_PROCESSOR);
        }
        
        // Si el heap estÃ¡ bajo, bajar prioridad de tareas no crÃ­ticas
        if (systemStatus.freeHeap < 100000) {  // Menos de 100KB libres
            vTaskPrioritySet(xWebServerHandle, TASK_PRIORITY_WEB_SERVER - 1);
            vTaskPrioritySet(xDisplayManagerHandle, TASK_PRIORITY_DISPLAY_MANAGER - 1);
        }
        
        lastAdjustment = now;
    }
}

// Monitor de tareas
void printTaskStatus() {
    Serial.println("\nðŸ“Š Estado de tareas:");
    Serial.println("====================");
    
    // InformaciÃ³n de cada tarea
    TaskStatus_t taskStatus;
    
    // Tarea SystemMonitor
    vTaskGetInfo(xSystemMonitorHandle, &taskStatus, pdTRUE, eInvalid);
    Serial.printf("SystemMonitor:  Estado=%d, Prio=%d\n", 
                  eTaskGetState(xSystemMonitorHandle), taskStatus.uxCurrentPriority);
    
    // Tarea SensorReader
    vTaskGetInfo(xSensorReaderHandle, &taskStatus, pdTRUE, eInvalid);
    Serial.printf("SensorReader:   Estado=%d, Prio=%d\n", 
                  eTaskGetState(xSensorReaderHandle), taskStatus.uxCurrentPriority);
    
    // Tarea MotorControl
    vTaskGetInfo(xMotorControlHandle, &taskStatus, pdTRUE, eInvalid);
    Serial.printf("MotorControl:   Estado=%d, Prio=%d\n", 
                  eTaskGetState(xMotorControlHandle), taskStatus.uxCurrentPriority);
    
    // Tarea AlarmProcessor
    vTaskGetInfo(xAlarmProcessorHandle, &taskStatus, pdTRUE, eInvalid);
    Serial.printf("AlarmProcessor: Estado=%d, Prio=%d\n", 
                  eTaskGetState(xAlarmProcessorHandle), taskStatus.uxCurrentPriority);
    
    // TamaÃ±os de cola
    Serial.printf("\nðŸ“¦ TamaÃ±os de cola:\n");
    Serial.printf("  Sensor: %d/%d\n", uxQueueMessagesWaiting(xSensorQueue), QUEUE_SENSOR_SIZE);
    Serial.printf("  Alarma: %d/%d\n", uxQueueMessagesWaiting(xAlarmQueue), QUEUE_ALARM_SIZE);
    Serial.printf("  Motor:  %d/%d\n", uxQueueMessagesWaiting(xMotorQueue), QUEUE_MOTOR_SIZE);
    Serial.printf("  Red:    %d/%d\n", uxQueueMessagesWaiting(xNetworkQueue), QUEUE_NETWORK_SIZE);
}

// Suspend/Resume de tareas
void suspendNonCriticalTasks() {
    // Suspender tareas no crÃ­ticas durante operaciones importantes
    vTaskSuspend(xWebServerHandle);
    vTaskSuspend(xDisplayManagerHandle);
    vTaskSuspend(xDataLoggerHandle);
    
    Serial.println("âš ï¸ Tareas no crÃ­ticas suspendidas");
}

void resumeNonCriticalTasks() {
    // Reanudar tareas suspendidas
    vTaskResume(xWebServerHandle);
    vTaskResume(xDisplayManagerHandle);
    vTaskResume(xDataLoggerHandle);
    
    Serial.println("âœ… Tareas no crÃ­ticas reanudadas");
}

// GestiÃ³n de watchdog
void initTaskWatchdogs() {
    // Configurar watchdog para tareas crÃ­ticas
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5000,  // 5 segundos
        .idle_core_mask = 0, // Monitorear ambos cores
        .trigger_panic = true
    };
    
    esp_task_wdt_reconfigure(&wdt_config);
    
    // Agregar tareas al watchdog
    esp_task_wdt_add(xSystemMonitorHandle);
    esp_task_wdt_add(xSensorReaderHandle);
    esp_task_wdt_add(xMotorControlHandle);
    esp_task_wdt_add(xAlarmProcessorHandle);
}

void feedTaskWatchdogs() {
    // Alimentar watchdog para cada tarea
    static uint32_t lastFeed = 0;
    
    if (millis() - lastFeed > 1000) {
        esp_task_wdt_reset();
        lastFeed = millis();
    }
}

// FunciÃ³n para manejar overflow de stack
void checkTaskStacks() {
    static uint32_t lastCheck = 0;
    
    if (millis() - lastCheck > 30000) {  // Cada 30 segundos
        UBaseType_t stackRemaining;
        
        // Verificar stack de tareas crÃ­ticas
        stackRemaining = uxTaskGetStackHighWaterMark(xSensorReaderHandle);
        if (stackRemaining < 100) {
            Serial.printf("âš ï¸ Stack bajo en SensorReader: %d bytes\n", stackRemaining);
        }
        
        stackRemaining = uxTaskGetStackHighWaterMark(xAlarmProcessorHandle);
        if (stackRemaining < 100) {
            Serial.printf("âš ï¸ Stack bajo en AlarmProcessor: %d bytes\n", stackRemaining);
        }
        
        lastCheck = millis();
    }
}

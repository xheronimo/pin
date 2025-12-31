// motor_controller.cpp - Control avanzado de motores
#include "system_config.h"
#include <PCF8574.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Expanders I2C para relays
PCF8574 relayExpander1(ADDR_RELAY_1);
PCF8574 relayExpander2(ADDR_RELAY_2);

// Estados de los motores
MotorStatus motor1 = {0};
MotorStatus motor2 = {0};

// Variables para control de velocidad
float motor1Speed = 0.0;
float motor2Speed = 0.0;
bool speedControlEnabled = false;

// Tiempos de alternancia
uint32_t motor1StartTime = 0;
uint32_t motor2StartTime = 0;

void initMotorController() {
    Serial.println("âš™ï¸ Inicializando controlador de motores...");
    
    // Inicializar expanders I2C
    if (!relayExpander1.begin() || !relayExpander2.begin()) {
        Serial.println("âŒ Error iniciando expanders de relays");
        return;
    }
    
    // Configurar pines de relays como salidas
    for (int i = 0; i < 8; i++) {
        relayExpander1.pinMode(i, OUTPUT);
        relayExpander2.pinMode(i, OUTPUT);
        relayExpander1.digitalWrite(i, LOW);
        relayExpander2.digitalWrite(i, LOW);
    }
    
    // Configurar pines para variador de velocidad (PWM)
    if (systemConfig.motorSpeedReference > 0) {
        initSpeedController();
        speedControlEnabled = true;
    }
    
    // Estado inicial
    motor1.state = MOTOR_OFF;
    motor2.state = MOTOR_OFF;
    motor1.autoMode = true;
    motor2.autoMode = true;
    
    Serial.println("âœ… Controlador de motores inicializado");
}

void startMotor(uint8_t motorId, uint8_t mode) {
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100))) {
        uint8_t relayPin = 0;
        bool success = false;
        
        if (motorId == 0) {  // Motor 1
            relayPin = 0;  // Pin 0 del expander 1
            relayExpander1.digitalWrite(relayPin, HIGH);
            motor1.state = MOTOR_STARTING;
            motor1StartTime = millis();
            motor1.startCount++;
            success = true;
            
            Serial.println("ðŸ”§ Motor 1 arrancando...");
        } 
        else if (motorId == 1) {  // Motor 2
            relayPin = 1;  // Pin 1 del expander 1
            relayExpander1.digitalWrite(relayPin, HIGH);
            motor2.state = MOTOR_STARTING;
            motor2StartTime = millis();
            motor2.startCount++;
            success = true;
            
            Serial.println("ðŸ”§ Motor 2 arrancando...");
        }
        
        // Si es modo automÃ¡tico, iniciar temporizador de arranque
        if (success && mode == MOTOR_AUTO) {
            // Crear timer para verificar arranque
            TimerHandle_t motorTimer = xTimerCreate(
                "MotorStartCheck",
                pdMS_TO_TICKS(2000),  // Verificar despuÃ©s de 2 segundos
                pdFALSE,
                (void*)motorId,
                motorStartCheckCallback
            );
            
            if (motorTimer != NULL) {
                xTimerStart(motorTimer, 0);
            }
        }
        
        xSemaphoreGive(xI2CMutex);
    }
}

void stopMotor(uint8_t motorId) {
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(100))) {
        uint8_t relayPin = 0;
        
        if (motorId == 0) {  // Motor 1
            relayPin = 0;
            relayExpander1.digitalWrite(relayPin, LOW);
            motor1.state = MOTOR_STOPPING;
            
            // Calcular horas de funcionamiento
            if (motor1StartTime > 0) {
                motor1.runningHours += (millis() - motor1StartTime) / 3600000;  // Horas
                motor1StartTime = 0;
            }
            
            Serial.println("ðŸ›‘ Motor 1 deteniÃ©ndose...");
        } 
        else if (motorId == 1) {  // Motor 2
            relayPin = 1;
            relayExpander1.digitalWrite(relayPin, LOW);
            motor2.state = MOTOR_STOPPING;
            
            if (motor2StartTime > 0) {
                motor2.runningHours += (millis() - motor2StartTime) / 3600000;
                motor2StartTime = 0;
            }
            
            Serial.println("ðŸ›‘ Motor 2 deteniÃ©ndose...");
        }
        
        xSemaphoreGive(xI2CMutex);
    }
}

void motorStartCheckCallback(TimerHandle_t xTimer) {
    uint8_t motorId = (uint8_t)pvTimerGetTimerID(xTimer);
    
    // Verificar si el motor arrancÃ³ correctamente
    // (en un sistema real, verificarÃ­as corriente, presiÃ³n, etc.)
    if (motorId == 0) {
        if (motor1.state == MOTOR_STARTING) {
            motor1.state = MOTOR_RUNNING;
            Serial.println("âœ… Motor 1 funcionando correctamente");
            
            // Enviar evento de arranque exitoso
            AlarmEvent alarm;
            alarm.priority = ALARM_LOW;
            strcpy(alarm.source, "MotorController");
            strcpy(alarm.message, "Motor 1 arrancado exitosamente");
            alarm.timestamp = millis();
            xQueueSend(xAlarmQueue, &alarm, 0);
        }
    } else if (motorId == 1) {
        if (motor2.state == MOTOR_STARTING) {
            motor2.state = MOTOR_RUNNING;
            Serial.println("âœ… Motor 2 funcionando correctamente");
            
            AlarmEvent alarm;
            alarm.priority = ALARM_LOW;
            strcpy(alarm.source, "MotorController");
            strcpy(alarm.message, "Motor 2 arrancado exitosamente");
            alarm.timestamp = millis();
            xQueueSend(xAlarmQueue, &alarm, 0);
        }
    }
}

void controlMotorSpeed(MotorStatus motors[2]) {
    if (!speedControlEnabled) return;
    
    // Control PID simplificado para velocidad de motor
    static float lastError1 = 0;
    static float integral1 = 0;
    static float lastError2 = 0;
    static float integral2 = 0;
    
    // ParÃ¡metros PID
    const float Kp = 0.8;
    const float Ki = 0.05;
    const float Kd = 0.1;
    
    if (motors[0].state == MOTOR_RUNNING) {
        // Leer velocidad actual (simulado - en realidad de sensor)
        float currentSpeed1 = getMotorSpeed(0);
        float error1 = systemConfig.motorSpeedReference - currentSpeed1;
        
        // CÃ¡lculo PID
        integral1 += error1;
        float derivative = error1 - lastError1;
        float output1 = Kp * error1 + Ki * integral1 + Kd * derivative;
        
        // Limitar salida
        output1 = constrain(output1, 0, 100);
        
        // Aplicar al variador
        setMotorSpeed(0, output1);
        motors[0].currentSpeed = output1;
        
        lastError1 = error1;
    }
    
    if (motors[1].state == MOTOR_RUNNING) {
        float currentSpeed2 = getMotorSpeed(1);
        float error2 = systemConfig.motorSpeedReference - currentSpeed2;
        
        integral2 += error2;
        float derivative = error2 - lastError2;
        float output2 = Kp * error2 + Ki * integral2 + Kd * derivative;
        
        output2 = constrain(output2, 0, 100);
        
        setMotorSpeed(1, output2);
        motors[1].currentSpeed = output2;
        
        lastError2 = error2;
    }
}

void automaticMotorAlternation(MotorStatus motors[2]) {
    if (!systemConfig.motorAlternationEnabled) return;
    
    static uint32_t lastAlternation = 0;
    uint32_t now = millis();
    
    // Verificar si es tiempo de alternar
    if (now - lastAlternation >= (systemConfig.motorAlternationTime * 1000)) {
        // LÃ³gica de alternancia basada en horas de funcionamiento
        if (motors[0].runningHours > motors[1].runningHours + 10) {
            // Motor 1 ha trabajado 10 horas mÃ¡s que el 2
            if (motors[0].state == MOTOR_RUNNING && motors[1].state == MOTOR_OFF) {
                // Detener motor 1, arrancar motor 2
                stopMotor(0);
                vTaskDelay(pdMS_TO_TICKS(2000));  // Esperar 2 segundos
                startMotor(1, MOTOR_AUTO);
                
                Serial.println("ðŸ”„ Alternancia: Motor 1 â†’ Motor 2");
                
                // Registrar evento
                AlarmEvent alarm;
                alarm.priority = ALARM_LOW;
                strcpy(alarm.source, "MotorController");
                strcpy(alarm.message, "Alternancia automÃ¡tica: Motor 1 â†’ Motor 2");
                alarm.timestamp = now;
                xQueueSend(xAlarmQueue, &alarm, 0);
            }
        } else if (motors[1].runningHours > motors[0].runningHours + 10) {
            if (motors[1].state == MOTOR_RUNNING && motors[0].state == MOTOR_OFF) {
                stopMotor(1);
                vTaskDelay(pdMS_TO_TICKS(2000));
                startMotor(0, MOTOR_AUTO);
                
                Serial.println("ðŸ”„ Alternancia: Motor 2 â†’ Motor 1");
                
                AlarmEvent alarm;
                alarm.priority = ALARM_LOW;
                strcpy(alarm.source, "MotorController");
                strcpy(alarm.message, "Alternancia automÃ¡tica: Motor 2 â†’ Motor 1");
                alarm.timestamp = now;
                xQueueSend(xAlarmQueue, &alarm, 0);
            }
        }
        
        lastAlternation = now;
    }
}

void processMotorCommand(MotorCommand cmd, MotorStatus motors[2]) {
    switch (cmd.command) {
        case MOTOR_START:
            if (cmd.motorId < 2) {
                startMotor(cmd.motorId, cmd.immediate ? MOTOR_MANUAL : MOTOR_AUTO);
                
                if (cmd.speed > 0) {
                    // Configurar velocidad si se especifica
                    setMotorSpeed(cmd.motorId, cmd.speed);
                    motors[cmd.motorId].currentSpeed = cmd.speed;
                }
            }
            break;
            
        case MOTOR_STOP:
            if (cmd.motorId < 2) {
                stopMotor(cmd.motorId);
            }
            break;
            
        case MOTOR_SET_SPEED:
            if (cmd.motorId < 2 && cmd.speed >= 0 && cmd.speed <= 100) {
                setMotorSpeed(cmd.motorId, cmd.speed);
                motors[cmd.motorId].currentSpeed = cmd.speed;
            }
            break;
            
        case MOTOR_TOGGLE:
            if (cmd.motorId < 2) {
                if (motors[cmd.motorId].state == MOTOR_RUNNING) {
                    stopMotor(cmd.motorId);
                } else {
                    startMotor(cmd.motorId, MOTOR_MANUAL);
                }
            }
            break;
    }
}

// Funciones auxiliares (simuladas para ejemplo)
void initSpeedController() {
    // Configurar PWM para variador de velocidad
    // En hardware real, configurarÃ­as los pines PWM
    Serial.println("ðŸŽšï¸ Control de velocidad inicializado");
}

float getMotorSpeed(uint8_t motorId) {
    // SimulaciÃ³n - en realidad leerÃ­as de sensor o variador
    return (motorId == 0) ? motor1Speed : motor2Speed;
}

void setMotorSpeed(uint8_t motorId, float speed) {
    // SimulaciÃ³n - en realidad enviarÃ­as seÃ±al al variador
    if (motorId == 0) {
        motor1Speed = speed;
    } else {
        motor2Speed = speed;
    }
    
    Serial.printf("âš¡ Velocidad Motor %d: %.1f%%\n", motorId + 1, speed);
}

MotorStatus getMotorStatus(uint8_t motorId) {
    return (motorId == 0) ? motor1 : motor2;
}

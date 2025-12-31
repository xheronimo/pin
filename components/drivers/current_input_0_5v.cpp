// input_strategies.cpp - Estrategias Ã³ptimas para entradas
#include "system_config.h"
#include <PCF8574.h>

// ==============================================
// ESTRATEGIA 1: CONMUTADOR DE POSICIONES (mejor en analÃ³gico)
// ==============================================

// Para conmutadores de mÃºltiples posiciones (3-8 posiciones), 
// es mejor usar UNA entrada analÃ³gica con divisor resistivo

typedef struct {
    uint8_t analogChannel;    // Canal analÃ³gico usado
    float voltageLevels[8];   // Niveles de voltaje para cada posiciÃ³n
    uint8_t positions;        // NÃºmero de posiciones (2-8)
    uint8_t currentPosition;
    uint8_t lastPosition;
    uint32_t debounceTime;
    uint32_t lastChange;
} MultiPositionSwitch;

MultiPositionSwitch modeSwitch = {
    .analogChannel = 15,      // Ãšltimo canal para no interferir con sensores
    .voltageLevels = {0.0, 1.1, 2.2, 3.3},  // 4 posiciones
    .positions = 4,
    .currentPosition = 0,
    .debounceTime = 50
};

// ==============================================
// ESTRATEGIA 2: ENTRADAS DIGITALES PURAS (marcha/paro)
// ==============================================

// Para botones simples (marcha, paro, reset), usar entradas digitales
// del PCF8574 o GPIOs directos

typedef struct {
    uint8_t gpioPin;          // Pin fÃ­sico (o pin en PCF8574)
    bool isPCF;              // True si estÃ¡ en expander
    uint8_t pcfAddress;      // DirecciÃ³n I2C del expander
    uint8_t pcfPin;          // Pin en el expander (0-7)
    bool pullUp;             // Resistencia pull-up interna
    bool inverted;           // LÃ³gica invertida
    bool currentState;
    bool lastState;
    uint32_t debounceTime;
    uint32_t lastChange;
} DigitalInput;

// ==============================================
// ESTRATEGIA 3: ENTRADAS DE SEGURIDAD (redundantes)
// ==============================================

// Para entradas crÃ­ticas (paro de emergencia), usar redundancia
// 2 entradas digitales + 1 analÃ³gica de verificaciÃ³n

typedef struct {
    DigitalInput primary;     // Entrada digital principal
    DigitalInput secondary;   // Entrada digital redundante
    uint8_t analogChannel;    // Canal analÃ³gico de verificaciÃ³n
    float analogThreshold;    // Umbral para estado activo
    bool safeState;          // Estado considerado "seguro"
    uint32_t faultTime;      // Tiempo mÃ¡ximo de divergencia
} SafetyInput;

// ==============================================
// IMPLEMENTACIÃ“N PRÃCTICA PARA TU SISTEMA
// ==============================================

// 1. CONMUTADOR DE MODO DE OPERACIÃ“N (4 posiciones)
//    Usar 1 entrada analÃ³gica
MultiPositionSwitch operationModeSwitch = {
    .analogChannel = 14,  // Canal analÃ³gico dedicado
    .voltageLevels = {
        0.0,    // PosiciÃ³n 0: OFF
        1.1,    // PosiciÃ³n 1: MANUAL
        2.2,    // PosiciÃ³n 2: AUTO
        3.3     // PosiciÃ³n 3: TEST
    },
    .positions = 4,
    .currentPosition = 1,  // Por defecto: MANUAL
    .debounceTime = 100
};

// 2. BOTONES DE CONTROL (marcha/paro/reset)
//    Usar entradas digitales del PCF8574
DigitalInput buttonStart = {
    .isPCF = true,
    .pcfAddress = ADDR_INPUT_1,
    .pcfPin = 0,      // Pin 0 del primer expander
    .pullUp = true,
    .inverted = true, // Normalmente abierto -> activo en LOW
    .debounceTime = 20
};

DigitalInput buttonStop = {
    .isPCF = true,
    .pcfAddress = ADDR_INPUT_1,
    .pcfPin = 1,
    .pullUp = true,
    .inverted = true,
    .debounceTime = 20
};

DigitalInput buttonReset = {
    .isPCF = true,
    .pcfAddress = ADDR_INPUT_1,
    .pcfPin = 2,
    .pullUp = true,
    .inverted = true,
    .debounceTime = 20
};

// 3. SELECTOR DE MOTOR (motor 1 / motor 2 / ambos)
//    Usar 2 entradas digitales para 3 estados:
//    - Motor 1: Pin4=1, Pin5=0
//    - Motor 2: Pin4=0, Pin5=1  
//    - Ambos:   Pin4=1, Pin5=1
//    - Ninguno: Pin4=0, Pin5=0
DigitalInput selectorMotor1 = {
    .isPCF = true,
    .pcfAddress = ADDR_INPUT_1,
    .pcfPin = 4,
    .pullUp = true,
    .inverted = false,
    .debounceTime = 50
};

DigitalInput selectorMotor2 = {
    .isPCF = true,
    .pcfAddress = ADDR_INPUT_1,
    .pcfPin = 5,
    .pullUp = true,
    .inverted = false,
    .debounceTime = 50
};

// 4. PARO DE EMERGENCIA (redundante)
SafetyInput emergencyStop = {
    .primary = {
        .isPCF = true,
        .pcfAddress = ADDR_INPUT_1,
        .pcfPin = 6,
        .pullUp = true,
        .inverted = true,  // Normalmente cerrado -> abierto en emergencia
        .debounceTime = 10
    },
    .secondary = {
        .isPCF = true,
        .pcfAddress = ADDR_INPUT_1,
        .pcfPin = 7,
        .pullUp = true,
        .inverted = true,
        .debounceTime = 10
    },
    .analogChannel = 13,  // VerificaciÃ³n analÃ³gica
    .analogThreshold = 1.0, // Voltaje mÃ­nimo para estado normal
    .safeState = false,   // FALSE = paro de emergencia activo
    .faultTime = 100      // 100ms mÃ¡ximo de divergencia
};

// ==============================================
// FUNCIONES DE LECTURA
// ==============================================

void initInputs() {
    Serial.println("ðŸŽ›ï¸ Inicializando sistema de entradas...");
    
    // Inicializar expanders PCF8574 para entradas
    initPCFInputs();
    
    // Configurar entradas analÃ³gicas para conmutadores
    analogReadResolution(12);  // 12 bits para mayor precisiÃ³n
    analogSetAttenuation(ADC_11db);  // 0-3.3V
    
    Serial.println("âœ… Sistema de entradas inicializado");
}

uint8_t readMultiPositionSwitch(MultiPositionSwitch &sw) {
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10))) {
        float voltage = readAnalogVoltage(sw.analogChannel);
        
        // Buscar posiciÃ³n mÃ¡s cercana
        uint8_t newPosition = 0;
        float minDifference = 3.3;  // MÃ¡xima diferencia posible
        
        for (uint8_t i = 0; i < sw.positions; i++) {
            float difference = fabs(voltage - sw.voltageLevels[i]);
            if (difference < minDifference) {
                minDifference = difference;
                newPosition = i;
            }
        }
        
        // Aplicar histÃ©resis y debounce
        if (newPosition != sw.currentPosition) {
            uint32_t now = millis();
            if (now - sw.lastChange > sw.debounceTime) {
                sw.lastPosition = sw.currentPosition;
                sw.currentPosition = newPosition;
                sw.lastChange = now;
                
                // Log del cambio
                Serial.printf("ðŸ”„ Conmutador cambiado: PosiciÃ³n %d â†’ %d (%.2fV)\n",
                             sw.lastPosition, newPosition, voltage);
                
                // Generar evento
                generateSwitchEvent(sw);
            }
        }
        
        xSemaphoreGive(xI2CMutex);
    }
    
    return sw.currentPosition;
}

bool readDigitalInput(DigitalInput &input) {
    bool state = false;
    
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(5))) {
        if (input.isPCF) {
            // Leer del expander PCF8574
            PCF8574 expander(input.pcfAddress);
            state = expander.digitalRead(input.pcfPin);
        } else {
            // Leer GPIO directo
            state = digitalRead(input.gpioPin);
        }
        
        // Aplicar lÃ³gica invertida si es necesario
        if (input.inverted) {
            state = !state;
        }
        
        // Debounce
        uint32_t now = millis();
        if (state != input.lastState) {
            if (now - input.lastChange > input.debounceTime) {
                input.lastState = input.currentState;
                input.currentState = state;
                input.lastChange = now;
                
                // Log solo si hay cambio real
                if (input.lastState != input.currentState) {
                    Serial.printf("ðŸ”˜ Entrada %d: %s â†’ %s\n", 
                                 input.pcfPin, 
                                 input.lastState ? "ON" : "OFF",
                                 input.currentState ? "ON" : "OFF");
                }
            }
        }
        
        xSemaphoreGive(xI2CMutex);
    }
    
    return input.currentState;
}

bool readSafetyInput(SafetyInput &safety) {
    bool primaryState = readDigitalInput(safety.primary);
    bool secondaryState = readDigitalInput(safety.secondary);
    
    // Verificar consistencia entre entradas redundantes
    if (primaryState != secondaryState) {
        // Lecturas divergentes - usar entrada analÃ³gica como desempate
        float analogVoltage = readAnalogVoltage(safety.analogChannel);
        bool analogState = (analogVoltage > safety.analogThreshold);
        
        // Determinar estado mÃ¡s probable
        int votes = 0;
        if (primaryState == analogState) votes++;
        if (secondaryState == analogState) votes++;
        
        if (votes >= 1) {
            // La mayorÃ­a coincide con analÃ³gica
            safety.primary.currentState = analogState;
            safety.secondary.currentState = analogState;
            
            // Generar alarma de inconsistencia
            static uint32_t lastFaultLog = 0;
            if (millis() - lastFaultLog > 5000) {
                Serial.printf("âš ï¸ Inconsistencia en entrada de seguridad: D1=%d, D2=%d, A=%.2fV\n",
                             primaryState, secondaryState, analogVoltage);
                lastFaultLog = millis();
            }
        }
    }
    
    // Estado seguro = ambas entradas en estado seguro
    return (safety.primary.currentState == safety.safeState && 
            safety.secondary.currentState == safety.safeState);
}

// ==============================================
// LÃ“GICA PARA SELECTORES COMBINADOS
// ==============================================

MotorSelection readMotorSelector() {
    bool m1 = readDigitalInput(selectorMotor1);
    bool m2 = readDigitalInput(selectorMotor2);
    
    if (m1 && !m2) {
        return MOTOR_SELECT_1;
    } else if (!m1 && m2) {
        return MOTOR_SELECT_2;
    } else if (m1 && m2) {
        return MOTOR_SELECT_BOTH;
    } else {
        return MOTOR_SELECT_NONE;
    }
}

OperationMode readOperationMode() {
    uint8_t position = readMultiPositionSwitch(operationModeSwitch);
    
    switch(position) {
        case 0: return MODE_OFF;
        case 1: return MODE_MANUAL;
        case 2: return MODE_AUTO;
        case 3: return MODE_TEST;
        default: return MODE_MANUAL;
    }
}

// ==============================================
// TAREA DE LECTURA DE ENTRADAS
// ==============================================

void vTaskInputReader(void *pvParameters) {
    Serial.println("ðŸŽ® Tarea InputReader iniciada");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 50Hz (20ms)
    
    // Variables para detecciÃ³n de pulsaciones largas
    uint32_t buttonHoldStart[3] = {0};  // Start, Stop, Reset
    bool buttonHeld[3] = {false};
    
    while (1) {
        // 1. Leer conmutador de modo (analÃ³gico)
        OperationMode currentMode = readOperationMode();
        if (currentMode != systemStatus.operationMode) {
            systemStatus.operationMode = currentMode;
            
            // Evento de cambio de modo
            AlarmEvent modeEvent;
            modeEvent.priority = ALARM_MEDIUM;
            strcpy(modeEvent.source, "InputSystem");
            snprintf(modeEvent.message, sizeof(modeEvent.message),
                    "Modo cambiado a: %s", 
                    getModeString(currentMode));
            xQueueSend(xAlarmQueue, &modeEvent, 0);
        }
        
        // 2. Leer botones (digitales)
        bool startPressed = readDigitalInput(buttonStart);
        bool stopPressed = readDigitalInput(buttonStop);
        bool resetPressed = readDigitalInput(buttonReset);
        
        // DetecciÃ³n de pulsaciones largas
        uint32_t now = millis();
        
        // BotÃ³n START pulsado largo (>3 segundos)
        if (startPressed && !buttonHeld[0]) {
            if (buttonHoldStart[0] == 0) {
                buttonHoldStart[0] = now;
            } else if (now - buttonHoldStart[0] > 3000) {
                buttonHeld[0] = true;
                Serial.println("ðŸ”„ START pulsaciÃ³n larga detectada");
                // AcciÃ³n especial: arranque forzado
                forceStartMotors();
            }
        } else if (!startPressed) {
            buttonHoldStart[0] = 0;
            buttonHeld[0] = false;
        }
        
        // BotÃ³n STOP pulsado largo (>5 segundos)
        if (stopPressed && !buttonHeld[1]) {
            if (buttonHoldStart[1] == 0) {
                buttonHoldStart[1] = now;
            } else if (now - buttonHoldStart[1] > 5000) {
                buttonHeld[1] = true;
                Serial.println("ðŸ›‘ STOP pulsaciÃ³n larga detectada - Bloqueo");
                // AcciÃ³n especial: bloqueo de sistema
                systemLockout();
            }
        } else if (!stopPressed) {
            buttonHoldStart[1] = 0;
            buttonHeld[1] = false;
        }
        
        // 3. Leer selector de motor
        MotorSelection motorSel = readMotorSelector();
        
        // 4. Leer paro de emergencia (redundante)
        bool emergencySafe = readSafetyInput(emergencyStop);
        if (!emergencySafe) {
            // Paro de emergencia activado
            emergencyStopProcedure();
        }
        
        // 5. Leer otras entradas digitales del segundo expander
        readAdditionalInputs();
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================
// RECOMENDACIONES DE ASIGNACIÃ“N PARA KCK868A16V3
// ==============================================

/*
ASIGNACIÃ“N Ã“PTIMA DE ENTRADAS:

PCF8574 #1 (0x21) - 8 entradas digitales:
  Pin 0: BotÃ³n START
  Pin 1: BotÃ³n STOP  
 # ContinuaciÃ³n del script - ConfiguraciÃ³n para entradas 0-5V
@"
// current_input_0_5v.cpp - ConfiguraciÃ³n para entradas 0-5V y 4-20mA
#include "system_config.h"

// ==============================================
// CONVERSIÃ“N 0-5V â†” 4-20mA - LA COMBINACIÃ“N PERFECTA
// ==============================================

/*
CON 0-5V ES MUCHO MÃS SIMPLE:

OpciÃ³n 1: Resistor 250Î© estÃ¡ndar
  4mA Ã— 250Î© = 1.0V
  20mA Ã— 250Î© = 5.0V
  â†’ Perfecto para 0-5V!

OpciÃ³n 2: Resistor 500Î© (mÃ¡s resoluciÃ³n en ADC)
  4mA Ã— 500Î© = 2.0V  
  20mA Ã— 500Î© = 10.0V
  â†’ Necesita divisor de voltaje 2:1 para 0-5V
*/

// ==============================================
// CIRCUITOS RECOMENDADOS
// ==============================================

/*
CIRCUITO 1: DIRECTO (Recomendado para KCK868A16V3)
----------------------------------------
Sensor 4-20mA â”€â”€â”¬â”€â”€ R=250Î© â”€â”€â”¬â”€â”€ 1-5V â”€â”€â†’ ADC 0-5V
                â”‚            â”‚
               GND          GND

NO necesita divisor, es directo 1:1
ESP32-S3 ADC: 0-3.3V, pero nuestra placa ya tiene acondicionamiento a 0-5V

CIRCUITO 2: CON PROTECCIÃ“N Y FILTRO
----------------------------------------
               +5V
                â”‚
                R1=10k
                â”‚
4-20mA â”€â”€â”€â”€â”¬â”€â”€â”€â”€â”´â”€â”€â”€â”€â”¬â”€â”€â”€â†’ ADC
           â”‚         â”‚
           C=100nF   R2=250Î©
           â”‚         â”‚
          GND       GND
*/

// ==============================================
// CONFIGURACIÃ“N ESPECÃFICA PARA 0-5V
// ==============================================

typedef struct {
    uint8_t channel;          // Canal analÃ³gico 0-15
    float resistorValue;      // Î© (0 para voltaje directo)
    float minVoltage;         // Voltaje mÃ­nimo esperado
    float maxVoltage;         // Voltaje mÃ¡ximo esperado
    float minCurrent;         // Corriente mÃ­nima (4mA)
    float maxCurrent;         // Corriente mÃ¡xima (20mA)
    float offsetVoltage;      // Offset en voltios (ej: 1V para 4mA)
    float scaleFactor;        // Factor de escala V/mA
    bool is4_20mA;           // True si es entrada 4-20mA
    bool hasDivider;         // True si tiene divisor de voltaje
    float dividerRatio;      // RelaciÃ³n del divisor (ej: 2.0 para 2:1)
    char sensorType[24];     // Tipo de sensor
    char units[16];          // Unidades de medida
} AnalogInputConfig;

AnalogInputConfig analogConfigs[16];

void initAnalogInputs0_5V() {
    Serial.println("ðŸ”Œ Configurando entradas 0-5V/4-20mA...");
    
    // ==============================================
    // CANALES 0-4: SENSORES DE PRESIÃ“N 4-20mA
    // ==============================================
    
    // Usando resistor de 250Î© directo a 0-5V
    for (int i = 0; i < 5; i++) {
        analogConfigs[i].channel = i;
        analogConfigs[i].resistorValue = 250.0;      // 250Î© estÃ¡ndar
        analogConfigs[i].minVoltage = 1.0;           // 4mA â†’ 1V
        analogConfigs[i].maxVoltage = 5.0;           // 20mA â†’ 5V
        analogConfigs[i].minCurrent = 4.0;           // 4mA
        analogConfigs[i].maxCurrent = 20.0;          // 20mA
        analogConfigs[i].offsetVoltage = 1.0;        // 1V a 4mA
        analogConfigs[i].scaleFactor = 0.25;         // 1V = 4mA, 0.25V/mA
        analogConfigs[i].is4_20mA = true;
        analogConfigs[i].hasDivider = false;
        analogConfigs[i].dividerRatio = 1.0;
        strcpy(analogConfigs[i].sensorType, "Pressure Sensor");
        strcpy(analogConfigs[i].units, "bar");
        
        Serial.printf("  Canal %d: PresiÃ³n %d - 4-20mA â†’ 1-5V (R=250Î©)\n", i, i+1);
    }
    
    // ==============================================
    // CANAL 5: VARIADOR DE VELOCIDAD 0-5V
    // ==============================================
    
    analogConfigs[5].channel = 5;
    analogConfigs[5].resistorValue = 0.0;            // Voltaje directo
    analogConfigs[5].minVoltage = 0.0;               // 0V
    analogConfigs[5].maxVoltage = 5.0;               // 5V
    analogConfigs[5].minCurrent = 0.0;
    analogConfigs[5].maxCurrent = 0.0;
    analogConfigs[5].offsetVoltage = 0.0;
    analogConfigs[5].scaleFactor = 1.0;              // 1:1
    analogConfigs[5].is4_20mA = false;
    analogConfigs[5].hasDivider = false;
    analogConfigs[5].dividerRatio = 1.0;
    strcpy(analogConfigs[5].sensorType, "VFD Speed Ref");
    strcpy(analogConfigs[5].units, "%");
    
    Serial.println("  Canal 5: Variador velocidad - 0-5V â†’ 0-100%");
    
    // ==============================================
    // CANALES 6-10: RESERVADOS/EXPANSIÃ“N
    // ==============================================
    
    for (int i = 6; i <= 10; i++) {
        analogConfigs[i].channel = i;
        analogConfigs[i].resistorValue = 250.0;      // Configurable vÃ­a jumper
        analogConfigs[i].minVoltage = 0.0;
        analogConfigs[i].maxVoltage = 5.0;
        analogConfigs[i].is4_20mA = true;           // Por defecto 4-20mA
        analogConfigs[i].hasDivider = false;
        analogConfigs[i].dividerRatio = 1.0;
        strcpy(analogConfigs[i].sensorType, "Reserved");
        strcpy(analogConfigs[i].units, "N/A");
        
        Serial.printf("  Canal %d: Reservado (configurable)\n", i);
    }
    
    // ==============================================
    // CANALES 11-15: ENTRADAS GENÃ‰RICAS 0-5V
    // ==============================================
    
    for (int i = 11; i < 16; i++) {
        analogConfigs[i].channel = i;
        analogConfigs[i].resistorValue = 0.0;        // Voltaje directo
        analogConfigs[i].minVoltage = 0.0;
        analogConfigs[i].maxVoltage = 5.0;
        analogConfigs[i].is4_20mA = false;
        analogConfigs[i].hasDivider = false;
        analogConfigs[i].dividerRatio = 1.0;
        strcpy(analogConfigs[i].sensorType, "Generic 0-5V");
        strcpy(analogConfigs[i].units, "V");
        
        Serial.printf("  Canal %d: GenÃ©rico 0-5V\n", i);
    }
    
    Serial.println("âœ… Entradas 0-5V/4-20mA configuradas");
}

// ==============================================
// CONVERSIÃ“N ADC â†’ VALOR REAL
// ==============================================

float readAnalogChannel(uint8_t channel) {
    if (channel >= 16) return 0.0;
    
    AnalogInputConfig &config = analogConfigs[channel];
    
    // 1. Leer ADC (ESP32-S3: 12-bit, 0-3.3V por defecto)
    // PERO: KCK868A16V3 ya tiene acondicionamiento a 0-5V â†’ 0-3.3V
    uint16_t rawADC = analogRead(getAnalogPin(channel));
    
    // 2. Convertir a voltaje (0-3.3V)
    float voltageADC = (rawADC / 4095.0) * 3.3;
    
    // 3. Si la placa tiene divisor 5Vâ†’3.3V, calcular voltaje original
    // Asumimos divisor 5Vâ†’3.3V = relaciÃ³n 5/3.3 â‰ˆ 1.515
    const float DIVIDER_RATIO = 5.0 / 3.3;  // 1.515
    
    float originalVoltage = voltageADC * DIVIDER_RATIO;
    
    // 4. Aplicar calibraciÃ³n del canal
    return calibrateInputValue(channel, originalVoltage, rawADC);
}

float calibrateInputValue(uint8_t channel, float voltage, uint16_t rawADC) {
    AnalogInputConfig &config = analogConfigs[channel];
    
    // 1. Para entradas 4-20mA
    if (config.is4_20mA) {
        // Convertir voltaje a corriente: I = V/R
        float current = (voltage / config.resistorValue) * 1000.0;  // mA
        
        // Aplicar lÃ­mites
        current = constrain(current, config.minCurrent, config.maxCurrent);
        
        // Convertir a valor de proceso (ej: 4-20mA â†’ 0-10 bar)
        // Escala lineal: process = ((I - 4) / 16) * span
        float span = config.maxCurrent - config.minCurrent;  // 16 mA
        float processValue = ((current - config.minCurrent) / span) * 
                            (config.maxVoltage - config.minVoltage);
        
        // Aplicar offset si es necesario
        processValue += config.minVoltage;
        
        return processValue;
    }
    
    // 2. Para entradas 0-5V directas
    // Aplicar calibraciÃ³n lineal
    float processValue = voltage;
    
    // Aplicar lÃ­mites
    processValue = constrain(processValue, config.minVoltage, config.maxVoltage);
    
    return processValue;
}

// ==============================================
// FUNCIONES DE CALIBRACIÃ“N Y DIAGNÃ“STICO
// ==============================================

void calibrate4_20mAChannel(uint8_t channel, float knownLow, float knownHigh) {
    if (channel >= 16 || !analogConfigs[channel].is4_20mA) return;
    
    Serial.printf("\nðŸ”§ Calibrando canal %d (4-20mA)...\n", channel);
    
    // 1. Solicitar aplicar 4mA
    Serial.println("  Aplicar 4mA al sensor y presionar ENTER");
    while (Serial.available() == 0) delay(100);
    Serial.readString();
    
    float voltageAt4mA = readAnalogChannel(channel);
    Serial.printf("  Lectura a 4mA: %.3fV\n", voltageAt4mA);
    
    // 2. Solicitar aplicar 20mA
    Serial.println("  Aplicar 20mA al sensor y presionar ENTER");
    while (Serial.available() == 0) delay(100);
    Serial.readString();
    
    float voltageAt20mA = readAnalogChannel(channel);
    Serial.printf("  Lectura a 20mA: %.3fV\n", voltageAt20mA);
    
    // 3. Calcular offset y escala
    analogConfigs[channel].offsetVoltage = voltageAt4mA;
    analogConfigs[channel].scaleFactor = (voltageAt20mA - voltageAt4mA) / 16.0;
    
    // 4. Actualizar resistor calculado
    float resistor = (voltageAt20mA - voltageAt4mA) / 0.016;  // 16mA = 0.016A
    analogConfigs[channel].resistorValue = resistor;
    
    Serial.printf("âœ… CalibraciÃ³n completada:\n");
    Serial.printf("   Offset: %.3fV, Escala: %.3fV/mA\n", 
                  analogConfigs[channel].offsetVoltage,
                  analogConfigs[channel].scaleFactor);
    Serial.printf("   Resistor calculado: %.1fÎ©\n", resistor);
}

// ==============================================
// DETECCIÃ“N DE FALLAS EN ENTRADAS 4-20mA
// ==============================================

typedef enum {
    INPUT_OK = 0,
    INPUT_UNDER_RANGE,      // < 3.5mA
    INPUT_OVER_RANGE,       // > 20.5mA
    INPUT_OPEN_CIRCUIT,     // < 1V (circuito abierto)
    INPUT_SHORT_CIRCUIT,    // > 5.5V (cortocircuito)
    INPUT_NOISE,           // Ruido excesivo
    INPUT_DRIFT           // Deriva de calibraciÃ³n
} InputFault;

InputFault checkInputFault(uint8_t channel) {
    if (channel >= 16) return INPUT_OPEN_CIRCUIT;
    
    AnalogInputConfig &config = analogConfigs[channel];
    float voltage = readAnalogChannel(channel);
    
    // 1. Circuito abierto (menos de 0.8V para 4-20mA)
    if (config.is4_20mA && voltage < 0.8) {
        return INPUT_OPEN_CIRCUIT;
    }
    
    // 2. Cortocircuito (mÃ¡s de 5.5V)
    if (voltage > 5.5) {
        return INPUT_SHORT_CIRCUIT;
    }
    
    // 3. Fuera de rango 4-20mA
    if (config.is4_20mA) {
        float current = (voltage / config.resistorValue) * 1000.0;
        
        if (current < 3.5) return INPUT_UNDER_RANGE;
        if (current > 20.5) return INPUT_OVER_RANGE;
    }
    
    // 4. Verificar ruido (variaciÃ³n rÃ¡pida)
    static float lastValues[16] = {0};
    static uint32_t lastCheck[16] = {0};
    
    uint32_t now = millis();
    if (now - lastCheck[channel] > 1000) {  // Cada segundo
        float variation = fabs(voltage - lastValues[channel]);
        if (variation > 0.5) {  // MÃ¡s de 0.5V de variaciÃ³n
            return INPUT_NOISE;
        }
        lastValues[channel] = voltage;
        lastCheck[channel] = now;
    }
    
    return INPUT_OK;
}

String getFaultString(InputFault fault) {
    switch(fault) {
        case INPUT_OK: return "OK";
        case INPUT_UNDER_RANGE: return "BAJO RANGO (<3.5mA)";
        case INPUT_OVER_RANGE: return "SOBRE RANGO (>20.5mA)";
        case INPUT_OPEN_CIRCUIT: return "CIRCUITO ABIERTO";
        case INPUT_SHORT_CIRCUIT: return "CORTOCIRCUITO";
        case INPUT_NOISE: return "RUIDO EXCESIVO";
        case INPUT_DRIFT: return "DERIVA DE CALIBRACIÃ“N";
        default: return "DESCONOCIDO";
    }
}

// ==============================================
// FILTRADO AVANZADO PARA SEÃ‘ALES ANALÃ“GICAS
// ==============================================

#define FILTER_WINDOW 10

typedef struct {
    float values[FILTER_WINDOW];
    uint8_t index;
    float sum;
    bool initialized;
} MovingAverageFilter;

MovingAverageFilter analogFilters[16];

float applyFilter(uint8_t channel, float newValue) {
    if (!analogFilters[channel].initialized) {
        // Inicializar filtro con el primer valor
        for (int i = 0; i < FILTER_WINDOW; i++) {
            analogFilters[channel].values[i] = newValue;
        }
        analogFilters[channel].sum = newValue * FILTER_WINDOW;
        analogFilters[channel].index = 0;
        analogFilters[channel].initialized = true;
        return newValue;
    }
    
    // Restar el valor mÃ¡s antiguo
    analogFilters[channel].sum -= analogFilters[channel].values[analogFilters[channel].index];
    
    // Agregar nuevo valor
    analogFilters[channel].values[analogFilters[channel].index] = newValue;
    analogFilters[channel].sum += newValue;
    
    // Actualizar Ã­ndice
    analogFilters[channel].index = (analogFilters[channel].index + 1) % FILTER_WINDOW;
    
    // Retornar promedio
    return analogFilters[channel].sum / FILTER_WINDOW;
}

// Filtro de mediana para eliminar spikes
float medianFilter(uint8_t channel, float newValue) {
    static float buffer[16][5];  // 5 valores por canal
    static uint8_t bufIndex[16] = {0};
    
    // Agregar nuevo valor al buffer
    buffer[channel][bufIndex[channel]] = newValue;
    bufIndex[channel] = (bufIndex[channel] + 1) % 5;
    
    // Copiar buffer para ordenar
    float temp[5];
    for (int i = 0; i < 5; i++) {
        temp[i] = buffer[channel][i];
    }
    
    // Ordenar burbuja simple
    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 5; j++) {
            if (temp[i] > temp[j]) {
                float swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }
    
    // Retornar mediana (posiciÃ³n 2 en array de 5)
    return temp[2];
}

// ==============================================
// LECTURA ROBUSTA CON VALIDACIÃ“N
// ==============================================

typedef struct {
    float value;
    uint32_t timestamp;
    uint8_t quality;        // 0-100%
    InputFault fault;
    bool valid;
} RobustReading;

RobustReading readAnalogChannelRobust(uint8_t channel) {
    RobustReading reading = {0};
    reading.timestamp = millis();
    
    // 1. Leer valor crudo
    float rawValue = readAnalogChannel(channel);
    
    // 2. Aplicar filtros
    float filteredValue = applyFilter(channel, rawValue);
    filteredValue = medianFilter(channel, filteredValue);
    
    // 3. Verificar fallas
    InputFault fault = checkInputFault(channel);
    reading.fault = fault;
    
    // 4. Calcular calidad
    if (fault == INPUT_OK) {
        reading.quality = 100;
        reading.valid = true;
        
        // Verificar deriva a largo plazo
        static float baseline[16] = {0};
        static uint32_t baselineTime[16] = {0};
        
        if (millis() - baselineTime[channel] > 300000) {  // 5 minutos
            float drift = fabs(filteredValue - baseline[channel]);
            if (drift > 0.1 && baseline[channel] != 0) {  // MÃ¡s de 0.1V de deriva
                reading.quality = 70;
                reading.fault = INPUT_DRIFT;
            }
            baseline[channel] = filteredValue;
            baselineTime[channel] = millis();
        }
    } else {
        reading.quality = 0;
        reading.valid = false;
        
        // Generar alarma de falla
        generateInputFaultAlarm(channel, fault, filteredValue);
    }
    
    reading.value = filteredValue;
    return reading;
}

void generateInputFaultAlarm(uint8_t channel, InputFault fault, float value) {
    static uint32_t lastAlarmTime[16] = {0};
    uint32_t now = millis();
    
    // Evitar alarmas repetitivas (mÃ­nimo 30 segundos)
    if (now - lastAlarmTime[channel] < 30000) return;
    
    AlarmEvent alarm;
    alarm.priority = (fault == INPUT_OPEN_CIRCUIT || fault == INPUT_SHORT_CIRCUIT) ? 
                     ALARM_HIGH : ALARM_MEDIUM;
    
    strcpy(alarm.source, "AnalogInput");
    snprintf(alarm.message, sizeof(alarm.message),
            "Canal %d (%s): %s - Valor: %.2fV",
            channel,
            analogConfigs[channel].sensorType,
            getFaultString(fault).c_str(),
            value);
    
    alarm.timestamp = now;
    
    // Para fallas graves, enviar SMS
    if (alarm.priority >= ALARM_HIGH) {
        strcpy(alarm.phoneNumbers[0], "+34600112233");
        alarm.smsCount = 1;
    }
    
    xQueueSend(xAlarmQueue, &alarm, 0);
    lastAlarmTime[channel] = now;
}

// ==============================================
// CONFIGURACIÃ“N PARA CADA TIPO DE SENSOR
// ==============================================

void configurePressureSensor(uint8_t channel, float minPressure, float maxPressure) {
    if (channel >= 16) return;
    
    analogConfigs[channel].minVoltage = 1.0;        // 4mA â†’ 1V
    analogConfigs[channel].maxVoltage = 5.0;        // 20mA â†’ 5V
    strcpy(analogConfigs[channel].sensorType, "Pressure Sensor");
    strcpy(analogConfigs[channel].units, "bar");
    
    // Configurar escalas para visualizaciÃ³n
    // 4-20mA â†’ 1-5V â†’ minPressure-maxPressure
    Serial.printf("ðŸ“Š Sensor presiÃ³n canal %d: %.1f-%.1f bar\n", 
                 channel, minPressure, maxPressure);
}

void configureLevelSensor(uint8_t channel, float minLevel, float maxLevel) {
    if (channel >= 16) return;
    
    analogConfigs[channel].minVoltage = 1.0;        // 4mA â†’ 1V
    analogConfigs[channel].maxVoltage = 5.0;        // 20mA â†’ 5V
    strcpy(analogConfigs[channel].sensorType, "Level Sensor");
    strcpy(analogConfigs[channel].units, "%");
    
    Serial.printf("ðŸ“Š Sensor nivel canal %d: %.1f-%.1f %%\n", 
                 channel, minLevel, maxLevel);
}

void configureTemperatureSensor(uint8_t channel, float minTemp, float maxTemp) {
    if (channel >= 16) return;
    
    // Asumir sensor RTD/termopar con transmisor 4-20mA
    analogConfigs[channel].minVoltage = 1.0;        // 4mA â†’ 1V
    analogConfigs[channel].maxVoltage = 5.0;        // 20mA â†’ 5V
    strcpy(analogConfigs[channel].sensorType, "Temperature Sensor");
    strcpy(analogConfigs[channel].units, "Â°C");
    
    Serial.printf("ðŸ“Š Sensor temperatura canal %d: %.1f-%.1f Â°C\n", 
                 channel, minTemp, maxTemp);
}

// ==============================================
// TAREA DE MONITOREO DE ENTRADAS ANALÃ“GICAS
// ==============================================

void vTaskAnalogMonitor(void *pvParameters) {
    Serial.println("ðŸ“ˆ Tarea AnalogMonitor iniciada");
    
    // Inicializar configuraciones
    initAnalogInputs0_5V();
    
    // Configurar sensores especÃ­ficos
    configurePressureSensor(0, 0.0, 10.0);    // PresiÃ³n 1: 0-10 bar
    configurePressureSensor(1, 0.0, 10.0);    // PresiÃ³n 2: 0-10 bar
    configurePressureSensor(2, 0.0, 10.0);    // PresiÃ³n 3: 0-10 bar
    configurePressureSensor(3, 0.0, 10.0);    // PresiÃ³n 4: 0-10 bar
    configurePressureSensor(4, 0.0, 10.0);    // PresiÃ³n 5: 0-10 bar
    configureLevelSensor(5, 0.0, 100.0);      // Variador: 0-100%
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);  // 10Hz
    
    // Buffer para tendencias
    float trendBuffer[16][60];  // 60 muestras por canal (6 segundos a 10Hz)
    uint8_t trendIndex[16] = {0};
    
    while (1) {
        // Leer y procesar cada canal activo
        for (int ch = 0; ch < 16; ch++) {
            // Saltar canales no configurados
            if (strcmp(analogConfigs[ch].sensorType, "Reserved") == 0 ||
                strcmp(analogConfigs[ch].sensorType, "Generic 0-5V") == 0) {
                continue;
            }
            
            // Lectura robusta
            RobustReading reading = readAnalogChannelRobust(ch);
            
            // Almacenar para tendencia
            trendBuffer[ch][trendIndex[ch]] = reading.value;
            trendIndex[ch] = (trendIndex[ch] + 1) % 60;
            
            // Crear dato de sensor para cola
            if (reading.valid) {
                SensorData data;
                data.type = (ch < 5) ? SENSOR_PRESSURE : 
                           (ch == 5) ? SENSOR_LEVEL : SENSOR_ANALOG;
                data.id = ch;
                data.value = reading.value;
                data.timestamp = reading.timestamp;
                data.quality = reading.quality;
                data.alarm = false;  // Se evalÃºa en otra tarea
                
                xQueueSend(xSensorQueue, &data, 0);
            }
            
            // Monitoreo de tendencias (cambios rÃ¡pidos)
            monitorTrends(ch, trendBuffer[ch], trendIndex[ch]);
        }
        
        // Log periÃ³dico de estado (cada 30 segundos)
        static uint32_t lastLog = 0;
        if (millis() - lastLog > 30000) {
            logAnalogStatus();
            lastLog = millis();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void monitorTrends(uint8_t channel, float* buffer, uint8_t index) {
    // Calcular tasa de cambio (derivada)
    float sum = 0;
    int count = 0;
    
    // Promedio de Ãºltimos 10 muestras
    for (int i = 0; i < 10 && i < 60; i++) {
        int idx = (index - i + 60) % 60;
        sum += buffer[idx];
        count++;
    }
    
    float avg = sum / count;
    
    // Comparar con promedio anterior
    static float lastAvg[16] = {0};
    float changeRate = fabs(avg - lastAvg[channel]);
    
    // Si cambio rÃ¡pido (>10% del rango por segundo)
    float range = analogConfigs[channel].maxVoltage - analogConfigs[channel].minVoltage;
    if (changeRate > (range * 0.1)) {
        // Posible problema o evento importante
        AlarmEvent alarm;
        alarm.priority = ALARM_MEDIUM;
        strcpy(alarm.source, "AnalogTrend");
        snprintf(alarm.message, sizeof(alarm.message),
                "Canal %d: Cambio rÃ¡pido %.1f%%/s", 
                channel, (changeRate/range)*100);
        alarm.timestamp = millis();
        
        xQueueSend(xAlarmQueue, &alarm, 0);
    }
    
    lastAvg[channel] = avg;
}

void logAnalogStatus() {
    Serial.println("\nðŸ“Š ESTADO ENTRADAS ANALÃ“GICAS:");
    Serial.println("CH | Tipo            | Valor   | Unid | Calidad | Estado");
    Serial.println("---|-----------------|---------|------|---------|--------");
    
    for (int ch = 0; ch < 16; ch++) {
        if (strcmp(analogConfigs[ch].sensorType, "Reserved") == 0) continue;
        
        RobustReading reading = readAnalogChannelRobust(ch);
        
        Serial.printf("%2d | %-15s | %7.2f | %-4s | %3d%%   | %s\n",
                     ch,
                     analogConfigs[ch].sensorType,
                     reading.value,
                     analogConfigs[ch].units,
                     reading.quality,
                     getFaultString(reading.fault).c_str());
    }
}

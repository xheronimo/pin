// modbus_config.cpp - ConfiguraciÃ³n especÃ­fica Modbus para medidores
#include "system_config.h"

// ==============================================
// CONFIGURACIÃ“N MEDIDOR DE POTENCIA TRIFÃSICO
// ==============================================

/*
EJEMPLO PARA MEDIDOR COMÃšN (EJ: SDM120, SDM630, PM210, ETC.)

Registros tÃ­picos Modbus (pueden variar segÃºn modelo):

Voltajes (V):
  0x0000: Voltaje L1-N (0.1V)   â†’ /10
  0x0001: Voltaje L2-N (0.1V)
  0x0002: Voltaje L3-N (0.1V)

Corrientes (A):
  0x0008: Corriente L1 (0.001A) â†’ /1000
  0x0009: Corriente L2
  0x000A: Corriente L3

Potencia Activa (W):
  0x0010: Potencia L1
  0x0011: Potencia L2  
  0x0012: Potencia L3

Potencia Reactiva (var):
  0x0018: Potencia Reactiva Total

Factor de Potencia:
  0x0019: Factor Potencia (0.001) â†’ /1000

Frecuencia (Hz):
  0x001A: Frecuencia (0.01Hz) â†’ /100

EnergÃ­a (kWh):
  0x0100-0x0101: EnergÃ­a Importada Total (32-bit)
*/

typedef struct {
    uint16_t voltage[3];      // Registros voltaje
    uint16_t current[3];      // Registros corriente
    uint16_t power[3];        // Registros potencia activa
    uint16_t reactivePower;   // Registro potencia reactiva
    uint16_t powerFactor;     // Registro factor potencia
    uint16_t frequency;       // Registro frecuencia
    uint16_t energy[2];       // Registros energÃ­a (32-bit)
    float voltageScale;       // Escala voltaje (ej: 0.1)
    float currentScale;       // Escala corriente (ej: 0.001)
    float powerScale;         // Escala potencia (ej: 1.0)
    float frequencyScale;     // Escala frecuencia (ej: 0.01)
} PowerMeterConfig;

PowerMeterConfig powerConfig = {
    .voltage = {0x0000, 0x0001, 0x0002},
    .current = {0x0008, 0x0009, 0x000A},
    .power = {0x0010, 0x0011, 0x0012},
    .reactivePower = 0x0018,
    .powerFactor = 0x0019,
    .frequency = 0x001A,
    .energy = {0x0100, 0x0101},
    .voltageScale = 0.1,      // 0.1V por unidad
    .currentScale = 0.001,    // 0.001A por unidad
    .powerScale = 1.0,        // 1W por unidad
    .frequencyScale = 0.01    // 0.01Hz por unidad
};

// ==============================================
// CONFIGURACIÃ“N SENSOR NIVEL MICROONDAS
// ==============================================

/*
EJEMPLO PARA SENSOR MICROONDAS TÃPICO (EJ: SIEMENS, VEGA, ETC.)

Registros comunes:
  0x0000: Nivel (0-10000 = 0-100%)
  0x0001: Distancia (mm)
  0x0002: Temperatura (0.1Â°C)
  0x0003: Calidad seÃ±al (0-100%)
  0x0004: Estado dispositivo
  0x0005: VersiÃ³n firmware
*/

typedef struct {
    uint16_t level;           // Registro nivel
    uint16_t distance;        // Registro distancia
    uint16_t temperature;     // Registro temperatura
    uint16_t signalQuality;   // Registro calidad seÃ±al
    uint16_t deviceStatus;    // Registro estado
    uint16_t firmware;        // Registro firmware
    float levelScale;         // Escala nivel (ej: 0.01 para %)
    float tempScale;          // Escala temperatura (ej: 0.1)
} LevelSensorConfig;

LevelSensorConfig levelConfig = {
    .level = 0x0000,
    .distance = 0x0001,
    .temperature = 0x0002,
    .signalQuality = 0x0003,
    .deviceStatus = 0x0004,
    .firmware = 0x0005,
    .levelScale = 0.01,       // 0.01 = /100 para porcentaje
    .tempScale = 0.1          // 0.1 = /10 para Â°C
};

// ==============================================
// FUNCIONES DE LECTURA GENÃ‰RICAS MODBUS
// ==============================================

float readModbusRegister(ModbusMaster& device, uint16_t registerAddr, float scale) {
    uint8_t result = device.readInputRegisters(registerAddr, 1);
    
    if (result == device.ku8MBSuccess) {
        return device.getResponseBuffer(0) * scale;
    } else {
        Serial.printf("Modbus error reg 0x%04X: %d\n", registerAddr, result);
        return NAN;
    }
}

uint32_t readModbusRegister32(ModbusMaster& device, uint16_t registerAddr) {
    uint8_t result = device.readInputRegisters(registerAddr, 2);
    
    if (result == device.ku8MBSuccess) {
        uint32_t value = (device.getResponseBuffer(0) << 16) | device.getResponseBuffer(1);
        return value;
    } else {
        Serial.printf("Modbus 32-bit error reg 0x%04X: %d\n", registerAddr, result);
        return 0;
    }
}

// ==============================================
// LECTURA COMPLETA MEDIDOR DE POTENCIA
// ==============================================

bool readPowerMeterComplete() {
    bool success = true;
    
    // Leer voltajes fase-fase
    for (int i = 0; i < 3; i++) {
        float voltage = readModbusRegister(powerMeter, powerConfig.voltage[i], powerConfig.voltageScale);
        if (!isnan(voltage)) {
            scadaData.powerData.voltage[i] = voltage;
        } else {
            success = false;
        }
    }
    
    // Leer corrientes
    for (int i = 0; i < 3; i++) {
        float current = readModbusRegister(powerMeter, powerConfig.current[i], powerConfig.currentScale);
        if (!isnan(current)) {
            scadaData.powerData.current[i] = current;
        } else {
            success = false;
        }
    }
    
    // Leer potencias activas por fase
    float totalPower = 0;
    for (int i = 0; i < 3; i++) {
        float power = readModbusRegister(powerMeter, powerConfig.power[i], powerConfig.powerScale);
        if (!isnan(power)) {
            scadaData.powerData.power[i] = power;
            totalPower += power;
        } else {
            success = false;
        }
    }
    scadaData.powerData.totalPower = totalPower;
    
    // Leer potencia reactiva total
    float reactive = readModbusRegister(powerMeter, powerConfig.reactivePower, powerConfig.powerScale);
    if (!isnan(reactive)) {
        scadaData.powerData.reactivePower = reactive;
    }
    
    // Leer factor de potencia
    float pf = readModbusRegister(powerMeter, powerConfig.powerFactor, 0.001);
    if (!isnan(pf)) {
        scadaData.powerData.powerFactor = pf;
    }
    
    // Leer frecuencia
    float freq = readModbusRegister(powerMeter, powerConfig.frequency, powerConfig.frequencyScale);
    if (!isnan(freq)) {
        scadaData.powerData.frequency = freq;
    }
    
    // Leer energÃ­a total (32-bit)
    uint32_t energy = readModbusRegister32(powerMeter, powerConfig.energy[0]);
    scadaData.powerData.energy = energy / 100.0; // Asumiendo 0.01kWh por unidad
    
    // Calcular potencia aparente
    scadaData.powerData.apparentPower = sqrt(
        pow(scadaData.powerData.totalPower, 2) + 
        pow(scadaData.powerData.reactivePower, 2)
    );
    
    return success;
}

// ==============================================
// LECTURA COMPLETA SENSOR DE NIVEL
// ==============================================

bool readLevelSensorComplete() {
    bool success = true;
    
    // Leer nivel (0-100%)
    float level = readModbusRegister(levelSensor, levelConfig.level, levelConfig.levelScale);
    if (!isnan(level)) {
        scadaData.levelData.level = constrain(level, 0.0, 100.0);
    } else {
        success = false;
        scadaData.levelData.valid = false;
    }
    
    // Leer distancia en mm
    float distance = readModbusRegister(levelSensor, levelConfig.distance, 1.0);
    if (!isnan(distance)) {
        scadaData.levelData.distance = distance;
    }
    
    // Leer temperatura del sensor
    float temp = readModbusRegister(levelSensor, levelConfig.temperature, levelConfig.tempScale);
    if (!isnan(temp)) {
        scadaData.levelData.temperature = temp;
    }
    
    // Leer calidad de seÃ±al
    float quality = readModbusRegister(levelSensor, levelConfig.signalQuality, 1.0);
    if (!isnan(quality)) {
        scadaData.levelData.signalQuality = constrain(quality, 0.0, 100.0);
    }
    
    // Leer estado del dispositivo
    uint16_t status = levelSensor.getResponseBuffer(0);
    scadaData.levelData.valid = (status == 0); // Asumiendo 0 = OK
    
    return success;
}

// ==============================================
// DIAGNÃ“STICO MODBUS
// ==============================================

void modbusDiagnostic() {
    Serial.println("\nðŸ” DIAGNÃ“STICO MODBUS");
    Serial.println("=====================");
    
    // Intentar leer registro de identificaciÃ³n de cada dispositivo
    Serial.println("1. MEDIDOR DE POTENCIA:");
    uint8_t result = powerMeter.readInputRegisters(0xFC00, 1); // Registro ID comÃºn
    if (result == powerMeter.ku8MBSuccess) {
        uint16_t deviceID = powerMeter.getResponseBuffer(0);
        Serial.printf("   âœ… Detectado - ID: 0x%04X\n", deviceID);
    } else {
        Serial.printf("   âŒ No responde - Error: %d\n", result);
    }
    
    Serial.println("\n2. SENSOR DE NIVEL:");
    result = levelSensor.readInputRegisters(0x0004, 1); // Registro estado
    if (result == levelSensor.ku8MBSuccess) {
        uint16_t status = levelSensor.getResponseBuffer(0);
        Serial.printf("   âœ… Detectado - Estado: 0x%04X\n", status);
        
        // Leer versiÃ³n firmware si estÃ¡ disponible
        result = levelSensor.readInputRegisters(levelConfig.firmware, 1);
        if (result == levelSensor.ku8MBSuccess) {
            uint16_t firmware = levelSensor.getResponseBuffer(0);
            Serial.printf("   Firmware: %d.%d\n", firmware >> 8, firmware & 0xFF);
        }
    } else {
        Serial.printf("   âŒ No responde - Error: %d\n", result);
    }
    
    // Verificar configuraciÃ³n de pines RS485
    Serial.println("\n3. CONFIGURACIÃ“N RS485:");
    Serial.printf("   TX Pin: GPIO%d\n", RS485_TX_PIN);
    Serial.printf("   RX Pin: GPIO%d\n", RS485_RX_PIN);
    Serial.printf("   DE Pin: GPIO%d\n", RS485_DE_PIN);
    Serial.printf("   Baudrate: 9600\n");
    Serial.printf("   Paridad: 8N1\n");
    
    // Test de comunicaciÃ³n bÃ¡sico
    Serial.println("\n4. TEST DE COMUNICACIÃ“N:");
    testModbusCommunication();
}

void testModbusCommunication() {
    uint32_t startTime = millis();
    int successCount = 0;
    int totalTests = 5;
    
    for (int i = 0; i < totalTests; i++) {
        if (readPowerMeterComplete()) {
            successCount++;
        }
        delay(200);
    }
    
    float successRate = (successCount * 100.0) / totalTests;
    Serial.printf("   Tasa Ã©xito: %.1f%% (%d/%d)\n", successRate, successCount, totalTests);
    
    if (successRate < 80.0) {
        Serial.println("   âš ï¸ Problemas de comunicaciÃ³n detectados");
        Serial.println("   Verificar:");
        Serial.println("   - Cableado RS485 (A/B/GND)");
        Serial.println("   - Direcciones Modbus");
        Serial.println("   - Terminadores de lÃ­nea");
        Serial.println("   - AlimentaciÃ³n dispositivos");
    } else {
        Serial.println("   âœ… ComunicaciÃ³n estable");
    }
}

// ==============================================
// CONFIGURACIÃ“N AVANZADA MODBUS
// ==============================================

void configureModbusAdvanced() {
    // Configurar timeout y reintentos
    powerMeter.setTimeout(1000);      // 1 segundo timeout
    powerMeter.setRetryCount(3);      // 3 reintentos
    
    levelSensor.setTimeout(1000);
    levelSensor.setRetryCount(3);
    
    // Configurar control DE pin mÃ¡s preciso
    powerMeter.preTransmission([]() {
        digitalWrite(RS485_DE_PIN, HIGH);
        delayMicroseconds(100);  // Tiempo para estabilizar
    });
    
    powerMeter.postTransmission([]() {
        delayMicroseconds(100);  // Ãšltimo bit transmitido
        digitalWrite(RS485_DE_PIN, LOW);
    });
    
    // Habilitar debug si es necesario
    #ifdef MODBUS_DEBUG
    powerMeter.setDebug(true);
    levelSensor.setDebug(true);
    #endif
    
    Serial.println("âš™ï¸ Modbus configurado en modo avanzado");
}

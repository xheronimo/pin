// multiplex_analog.cpp - Lectura de 16 entradas analÃ³gicas multiplexadas
#include "system_config.h"
#include <PCF8574.h>
#include <freertos/FreeRTOS.h>

// Expander para control de multiplexor
PCF8574 analogMuxExpander(ADDR_INPUT_1);

// Tabla de calibraciÃ³n para cada canal (4-20mA -> valor de proceso)
typedef struct {
    float zeroValue;     // Valor a 4mA
    float spanValue;     // Valor a 20mA
    float minProcess;    // Valor mÃ­nimo del proceso
    float maxProcess;    // Valor mÃ¡ximo del proceso
    char units[16];      // Unidades de medida
    bool enabled;        // Canal habilitado
} ChannelCalibration;

ChannelCalibration channelCal[16];

// Buffer circular para promediar lecturas
#define SAMPLE_BUFFER_SIZE 10
float sampleBuffer[16][SAMPLE_BUFFER_SIZE];
uint8_t sampleIndex[16] = {0};

void initAnalogMultiplexer() {
    Serial.println("ðŸ“Š Inicializando multiplexor analÃ³gico...");
    
    if (!analogMuxExpander.begin()) {
        Serial.println("âŒ Error iniciando expander para multiplexor");
        return;
    }
    
    // Configurar pines de control del multiplexor como salidas
    for (int i = 0; i < 4; i++) {  // Se necesitan 4 bits para 16 canales
        analogMuxExpander.pinMode(i, OUTPUT);
        analogMuxExpander.digitalWrite(i, LOW);
    }
    
    // Configurar calibraciÃ³n por defecto para cada canal
    for (int ch = 0; ch < 16; ch++) {
        channelCal[ch].zeroValue = 0.0;
        channelCal[ch].spanValue = 100.0;
        channelCal[ch].minProcess = 0.0;
        channelCal[ch].maxProcess = 100.0;
        strcpy(channelCal[ch].units, "mA");
        channelCal[ch].enabled = true;
        
        // Inicializar buffer de muestras
        for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
            sampleBuffer[ch][i] = 0.0;
        }
        sampleIndex[ch] = 0;
    }
    
    // Configurar canales especÃ­ficos para sensores de presiÃ³n
    // Canal 0-4: Sensores de presiÃ³n (4-20mA -> 0-10 bar)
    for (int i = 0; i < 5; i++) {
        channelCal[i].zeroValue = 0.0;      // 4mA = 0 bar
        channelCal[i].spanValue = 10.0;     // 20mA = 10 bar
        channelCal[i].minProcess = 0.0;
        channelCal[i].maxProcess = 10.0;
        strcpy(channelCal[i].units, "bar");
    }
    
    Serial.println("âœ… Multiplexor analÃ³gico inicializado (16 canales)");
}

void selectAnalogChannel(uint8_t channel) {
    if (channel >= 16) return;
    
    // Controlar multiplexor con 4 bits
    analogMuxExpander.digitalWrite(0, (channel & 0x01) ? HIGH : LOW);
    analogMuxExpander.digitalWrite(1, (channel & 0x02) ? HIGH : LOW);
    analogMuxExpander.digitalWrite(2, (channel & 0x04) ? HIGH : LOW);
    analogMuxExpander.digitalWrite(3, (channel & 0x08) ? HIGH : LOW);
    
    // PequeÃ±o delay para estabilizaciÃ³n
    delayMicroseconds(50);
}

uint8_t getAnalogPin(uint8_t channel) {
    // Mapear canales a pines ADC fÃ­sicos
    // En KCK868A16V3, hay 4 entradas ADC multiplexadas
    static const uint8_t pinMapping[16] = {
        PIN_ANALOG_A1, PIN_ANALOG_A1, PIN_ANALOG_A1, PIN_ANALOG_A1,  // Canal 0-3 en A1
        PIN_ANALOG_A2, PIN_ANALOG_A2, PIN_ANALOG_A2, PIN_ANALOG_A2,  // Canal 4-7 en A2
        PIN_ANALOG_A3, PIN_ANALOG_A3, PIN_ANALOG_A3, PIN_ANALOG_A3,  // Canal 8-11 en A3
        PIN_ANALOG_A4, PIN_ANALOG_A4, PIN_ANALOG_A4, PIN_ANALOG_A4   // Canal 12-15 en A4
    };
    
    return (channel < 16) ? pinMapping[channel] : PIN_ANALOG_A1;
}

float readAnalogChannel(uint8_t channel) {
    if (!channelCal[channel].enabled) return 0.0;
    
    // Seleccionar canal en multiplexor
    selectAnalogChannel(channel);
    
    // Leer valor ADC
    uint16_t adcValue = analogRead(getAnalogPin(channel));
    
    // Convertir a voltaje (ESP32-S3: 12-bit ADC, 0-3.3V)
    float voltage = (adcValue / 4095.0) * 3.3;
    
    // Convertir a corriente (4-20mA) usando resistor de 250 ohm
    float current = (voltage / 250.0) * 1000.0;  // mA
    
    // Filtrar ruido (eliminar valores fuera de rango)
    if (current < 3.0 || current > 21.0) {
        // Valor fuera de rango, usar Ãºltima lectura vÃ¡lida
        return getLastValidReading(channel);
    }
    
    // CalibraciÃ³n lineal 4-20mA -> valor de proceso
    float processValue = calibrateReading(current, channel);
    
    // Aplicar filtro de promedio mÃ³vil
    processValue = applyMovingAverage(channel, processValue);
    
    // Detectar cambios significativos
    checkForSignificantChange(channel, processValue);
    
    return processValue;
}

float calibrateReading(float current, uint8_t channel) {
    // ConversiÃ³n lineal: 4-20mA -> valor de proceso
    float processValue = channelCal[channel].zeroValue + 
                        ((current - 4.0) / 16.0) * channelCal[channel].spanValue;
    
    // Aplicar lÃ­mites
    processValue = constrain(processValue, 
                            channelCal[channel].minProcess, 
                            channelCal[channel].maxProcess);
    
    return processValue;
}

float applyMovingAverage(uint8_t channel, float newValue) {
    // Agregar nueva muestra al buffer
    sampleBuffer[channel][sampleIndex[channel]] = newValue;
    sampleIndex[channel] = (sampleIndex[channel] + 1) % SAMPLE_BUFFER_SIZE;
    
    // Calcular promedio
    float sum = 0.0;
    uint8_t validSamples = 0;
    
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        if (sampleBuffer[channel][i] != 0.0) {
            sum += sampleBuffer[channel][i];
            validSamples++;
        }
    }
    
    return (validSamples > 0) ? (sum / validSamples) : newValue;
}

float getLastValidReading(uint8_t channel) {
    // Buscar Ãºltima lectura vÃ¡lida en buffer
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        int idx = (sampleIndex[channel] - 1 - i + SAMPLE_BUFFER_SIZE) % SAMPLE_BUFFER_SIZE;
        if (sampleBuffer[channel][idx] != 0.0) {
            return sampleBuffer[channel][idx];
        }
    }
    return 0.0;
}

void checkForSignificantChange(uint8_t channel, float newValue) {
    static float lastValues[16] = {0};
    static uint32_t lastChangeTime[16] = {0};
    
    float delta = fabs(newValue - lastValues[channel]);
    float threshold = (channelCal[channel].maxProcess - channelCal[channel].minProcess) * 0.02;  // 2% del rango
    
    if (delta > threshold) {
        // Cambio significativo detectado
        uint32_t now = millis();
        
        if (now - lastChangeTime[channel] > 1000) {  // Evitar flood (1 segundo mÃ­nimo)
            Serial.printf("ðŸ“ˆ Cambio significativo en canal %d: %.2f â†’ %.2f %s\n", 
                         channel, lastValues[channel], newValue, channelCal[channel].units);
            
            // Crear evento de cambio
            SensorData data;
            data.type = SENSOR_ANALOG;
            data.id = channel;
            data.value = newValue;
            data.timestamp = now;
            data.quality = 100;
            data.alarm = false;
            
            xQueueSend(xSensorQueue, &data, 0);
            
            lastChangeTime[channel] = now;
        }
    }
    
    lastValues[channel] = newValue;
}

void readAllAnalogChannels() {
    static uint32_t lastScan = 0;
    uint32_t now = millis();
    
    if (now - lastScan >= 100) {  // Escanear cada 100ms
        for (uint8_t ch = 0; ch < 16; ch++) {
            if (channelCal[ch].enabled) {
                float value = readAnalogChannel(ch);
                
                // Verificar alarmas
                checkAnalogAlarms(ch, value);
            }
        }
        
        lastScan = now;
    }
}

void checkAnalogAlarms(uint8_t channel, float value) {
    // Umbrales de alarma por canal
    static float alarmHigh[16] = {0};
    static float alarmLow[16] = {0};
    
    // Configurar umbrales para sensores de presiÃ³n
    if (channel < 5) {  // Canales 0-4: sensores de presiÃ³n
        alarmHigh[channel] = 8.0;  // 8 bar
        alarmLow[channel] = 1.0;   // 1 bar
    }
    
    // Verificar alarmas
    if (value > alarmHigh[channel]) {
        triggerAnalogAlarm(channel, value, ALARM_HIGH, "ALTO");
    } else if (value < alarmLow[channel]) {
        triggerAnalogAlarm(channel, value, ALARM_HIGH, "BAJO");
    }
}

void triggerAnalogAlarm(uint8_t channel, float value, AlarmPriority priority, const char* type) {
    static uint32_t lastAlarmTime[16] = {0};
    uint32_t now = millis();
    
    // Evitar alarmas repetidas muy seguidas
    if (now - lastAlarmTime[channel] > 5000) {  // 5 segundos mÃ­nimo entre alarmas
        AlarmEvent alarm;
        alarm.priority = priority;
        snprintf(alarm.source, sizeof(alarm.source), "AnalogCh%d", channel);
        snprintf(alarm.message, sizeof(alarm.message), 
                "Canal %d: %.2f %s - Nivel %s", 
                channel, value, channelCal[channel].units, type);
        alarm.timestamp = now;
        
        // Configurar nÃºmeros para SMS (ejemplo)
        strcpy(alarm.phoneNumbers[0], "+34600112233");
        alarm.smsCount = 1;
        
        xQueueSend(xAlarmQueue, &alarm, 0);
        
        lastAlarmTime[channel] = now;
        
        Serial.printf("ðŸš¨ Alarma canal %d: %s\n", channel, alarm.message);
    }
}

void calibrateChannel(uint8_t channel, float zeroValue, float spanValue) {
    if (channel >= 16) return;
    
    channelCal[channel].zeroValue = zeroValue;
    channelCal[channel].spanValue = spanValue;
    
    Serial.printf("ðŸ”§ CalibraciÃ³n canal %d: Zero=%.2f, Span=%.2f\n", 
                  channel, zeroValue, spanValue);
}

ChannelCalibration getChannelCalibration(uint8_t channel) {
    return (channel < 16) ? channelCal[channel] : ChannelCalibration();
}

void setChannelEnabled(uint8_t channel, bool enabled) {
    if (channel < 16) {
        channelCal[channel].enabled = enabled;
    }
}

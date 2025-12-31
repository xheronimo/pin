// json_system_scada.cpp - Sistema JSON para SCADA con FreeRTOS
#include "system_config.h"
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ==============================================
// CONFIGURACIÃ“N DE ENTRADAS DIGITALES ACTUALIZADA
// ==============================================

/*
ENTRADAS DIGITALES PCF8574 #1 (0x21) - 8 entradas:
  D0: Alarma Motor 1                    (NC - Normalmente Cerrado)
  D1: Alarma Motor 2                    (NC)
  D2: Funcionamiento Motor 1 AutomÃ¡tico (NA - Normalmente Abierto)  
  D3: Funcionamiento Motor 1 Manual     (NA)
  D4: Funcionamiento Motor 2 AutomÃ¡tico (NA)
  D5: Funcionamiento Motor 2 Manual     (NA)
  D6: Selector Motor 1 Auto/Manual      (0=Manual, 1=Auto)
  D7: Selector Motor 2 Auto/Manual      (0=Manual, 1=Auto)

ENTRADAS DIGITALES PCF8574 #2 (0x22) - 8 entradas:
  D8:  Selector Contactores/Variador    (0=Contactores, 1=Variador)
  D9:  Reservado / ExpansiÃ³n
  D10: Reservado / ExpansiÃ³n
  D11: Reservado / ExpansiÃ³n
  D12: Paro Emergencia                  (NC - Normalmente Cerrado)
  D13: Reset Alarmas                    (NA - Pulsador)
  D14: Marcha General                   (NA - Pulsador)
  D15: Paro General                     (NA - Pulsador)
*/

typedef enum {
    // PCF8574 #1
    ALARMA_MOTOR_1 = 0,
    ALARMA_MOTOR_2 = 1,
    FUNC_M1_AUTO = 2,
    FUNC_M1_MANUAL = 3,
    FUNC_M2_AUTO = 4,
    FUNC_M2_MANUAL = 5,
    SEL_M1_AUTO_MAN = 6,
    SEL_M2_AUTO_MAN = 7,
    
    // PCF8574 #2
    SEL_CONT_VARIADOR = 8,
    RESERVADO_1 = 9,
    RESERVADO_2 = 10,
    RESERVADO_3 = 11,
    PARO_EMERGENCIA = 12,
    RESET_ALARMAS = 13,
    MARCHA_GENERAL = 14,
    PARO_GENERAL = 15
} DigitalInputID;

// ==============================================
// CONFIGURACIÃ“N SERVIDOR EXTERNO
// ==============================================

typedef struct {
    char serverUrl[128];        // URL del servidor JSON
    char apiKey[64];           // API Key para autenticaciÃ³n
    uint32_t updateInterval;   // Intervalo de envÃ­o (ms)
    uint8_t retryCount;        // NÃºmero de reintentos
    uint32_t timeout;          // Timeout de conexiÃ³n (ms)
    bool enableCompression;    // Habilitar compresiÃ³n gzip
    bool enableEncryption;     // Habilitar encriptaciÃ³n
    bool sendOnlyChanges;      // Enviar solo cambios
    float changeThreshold;     // Umbral para considerar cambio
    bool enableDiagnostics;    // Enviar datos de diagnÃ³stico
} ServerConfig;

ServerConfig externalServer = {
    .serverUrl = "http://tuserver.com/api/scada",
    .apiKey = "TU_API_KEY_AQUI",
    .updateInterval = 10000,    // 10 segundos
    .retryCount = 3,
    .timeout = 5000,
    .enableCompression = true,
    .enableEncryption = false,
    .sendOnlyChanges = true,
    .changeThreshold = 0.5,     // 0.5% de cambio
    .enableDiagnostics = true
};

// ==============================================
// VARIABLES PARA DETECCIÃ“N DE CAMBIOS
// ==============================================

typedef struct {
    float lastValue;
    uint32_t lastSendTime;
    bool needsUpdate;
    float changeThreshold;
} SensorChangeTracker;

SensorChangeTracker sensorChanges[16];
bool digitalChanges[16] = {false};
MotorStatus lastMotorStatus[2];

// ==============================================
// FORMATOS JSON PARA DIFERENTES PROPÃ“SITOS
// ==============================================

String generateCompactJsonForServer() {
    /*
    Formato compacto para servidor externo
    Solo datos esenciales, sin formato bonito
    */
    JsonDocument doc;
    
    // Header con metadatos
    doc["device_id"] = systemConfig.deviceID;
    doc["timestamp"] = millis();
    doc["uptime"] = systemStatus.uptime;
    doc["fw_version"] = "1.0.0";
    
    // Sensores analÃ³gicos (solo si cambiaron)
    JsonArray sensors = doc["s"].to<JsonArray>();
    for (int i = 0; i < 16; i++) {
        if (sensorChanges[i].needsUpdate) {
            JsonObject sensor = sensors.add<JsonObject>();
            sensor["id"] = i;
            sensor["v"] = getCurrentSensorValue(i);  // Valor
            sensor["q"] = getSensorQuality(i);       // Calidad
            sensorChanges[i].needsUpdate = false;
        }
    }
    
    // Entradas digitales (solo si cambiaron)
    JsonArray digital = doc["d"].to<JsonArray>();
    for (int i = 0; i < 16; i++) {
        if (digitalChanges[i]) {
            JsonObject dig = digital.add<JsonObject>();
            dig["id"] = i;
            dig["s"] = digitalInputs[i].currentState;  // Estado
            digitalChanges[i] = false;
        }
    }
    
    // Estado de motores (siempre enviar, es crÃ­tico)
    JsonObject motors = doc["m"].to<JsonObject>();
    motors["m1"]["st"] = motor1.state;          // Estado motor 1
    motors["m1"]["sp"] = motor1Speed;          // Velocidad
    motors["m1"]["hr"] = motor1.runningHours;  // Horas trabajo
    motors["m2"]["st"] = motor2.state;
    motors["m2"]["sp"] = motor2Speed;
    motors["m2"]["hr"] = motor2.runningHours;
    
    // Modo de operaciÃ³n
    doc["mode"] = getOperationModeString();
    
    // Alarmas activas
    JsonArray alarms = doc["a"].to<JsonArray>();
    // ... agregar alarmas activas ...
    
    String output;
    serializeJson(doc, output);
    return output;
}

String generateFullJsonForWeb() {
    /*
    Formato completo para pÃ¡gina web
    Con nombres descriptivos y formato legible
    */
    JsonDocument doc;
    
    // InformaciÃ³n del sistema
    doc["system"]["device_id"] = systemConfig.deviceID;
    doc["system"]["location"] = "DepÃ³sito de Agua";
    doc["system"]["timestamp"] = getFormattedTime();
    doc["system"]["uptime"] = formatUptime(systemStatus.uptime);
    doc["system"]["free_heap"] = systemStatus.freeHeap;
    doc["system"]["temperature"] = systemStatus.temperature;
    
    // Sensores analÃ³gicos - PRESIÃ“N (0-4)
    JsonArray pressureSensors = doc["sensors"]["pressure"].to<JsonArray>();
    for (int i = 0; i < 5; i++) {
        JsonObject sensor = pressureSensors.add<JsonObject>();
        sensor["id"] = i + 1;
        sensor["name"] = String("PresiÃ³n ") + (i + 1);
        sensor["value"] = getCurrentSensorValue(i);
        sensor["units"] = "bar";
        sensor["min"] = 0.0;
        sensor["max"] = 10.0;
        sensor["alarm_low"] = systemConfig.pressureAlarmLow;
        sensor["alarm_high"] = systemConfig.pressureAlarmHigh;
        sensor["quality"] = getSensorQuality(i);
        sensor["status"] = getSensorStatus(i);
    }
    
    // Variador de velocidad (canal 5)
    doc["sensors"]["vfd"]["value"] = getCurrentSensorValue(5);
    doc["sensors"]["vfd"]["units"] = "%";
    doc["sensors"]["vfd"]["setpoint"] = systemConfig.motorSpeedReference;
    
    // Nivel de agua (RS485)
    doc["sensors"]["water_level"]["value"] = getWaterLevel();
    doc["sensors"]["water_level"]["units"] = "%";
    doc["sensors"]["water_level"]["alarm_low"] = systemConfig.levelAlarmLow;
    doc["sensors"]["water_level"]["alarm_high"] = systemConfig.levelAlarmHigh;
    
    // Medidor de potencia (I2C)
    doc["sensors"]["power"]["voltage_l1"] = cfgApp.tension[0].estado.valor;
    doc["sensors"]["power"]["voltage_l2"] = cfgApp.tension[1].estado.valor;
    doc["sensors"]["power"]["voltage_l3"] = cfgApp.tension[2].estado.valor;
    doc["sensors"]["power"]["current_l1"] = cfgApp.intensidad[0].estado.valor;
    doc["sensors"]["power"]["current_l2"] = cfgApp.intensidad[1].estado.valor;
    doc["sensors"]["power"]["current_l3"] = cfgApp.intensidad[2].estado.valor;
    doc["sensors"]["power"]["active_power"] = calculateActivePower();
    doc["sensors"]["power"]["reactive_power"] = calculateReactivePower();
    doc["sensors"]["power"]["power_factor"] = calculatePowerFactor();
    
    // Temperatura y humedad (SHT31)
    doc["sensors"]["environment"]["temperature"] = cfgApp.temperatura.estado.valor;
    doc["sensors"]["environment"]["humidity"] = cfgApp.humedad.estado.valor;
    
    // ==============================================
    // ENTRADAS DIGITALES - CON NOMBRES DESCRIPTIVOS
    // ==============================================
    
    JsonObject digitalInputsObj = doc["digital_inputs"].to<JsonObject>();
    
    // Grupo: Alarmas
    JsonObject alarmsGroup = digitalInputsObj["alarms"].to<JsonObject>();
    alarmsGroup["motor_1"] = {
        {"state", digitalInputs[ALARMA_MOTOR_1].currentState},
        {"description", "Alarma Motor 1"},
        {"normal_state", "NC (Normalmente Cerrado)"},
        {"active", digitalInputs[ALARMA_MOTOR_1].currentState == false}
    };
    
    alarmsGroup["motor_2"] = {
        {"state", digitalInputs[ALARMA_MOTOR_2].currentState},
        {"description", "Alarma Motor 2"},
        {"normal_state", "NC"},
        {"active", digitalInputs[ALARMA_MOTOR_2].currentState == false}
    };
    
    // Grupo: Funcionamiento
    JsonObject operationGroup = digitalInputsObj["operation"].to<JsonObject>();
    operationGroup["motor_1_auto"] = {
        {"state", digitalInputs[FUNC_M1_AUTO].currentState},
        {"description", "Func. Motor 1 AutomÃ¡tico"},
        {"normal_state", "NA (Normalmente Abierto)"}
    };
    
    operationGroup["motor_1_manual"] = {
        {"state", digitalInputs[FUNC_M1_MANUAL].currentState},
        {"description", "Func. Motor 1 Manual"},
        {"normal_state", "NA"}
    };
    
    operationGroup["motor_2_auto"] = {
        {"state", digitalInputs[FUNC_M2_AUTO].currentState},
        {"description", "Func. Motor 2 AutomÃ¡tico"},
        {"normal_state", "NA"}
    };
    
    operationGroup["motor_2_manual"] = {
        {"state", digitalInputs[FUNC_M2_MANUAL].currentState},
        {"description", "Func. Motor 2 Manual"},
        {"normal_state", "NA"}
    };
    
    // Grupo: Selectores
    JsonObject selectorsGroup = digitalInputsObj["selectors"].to<JsonObject>();
    selectorsGroup["motor_1_auto_man"] = {
        {"state", digitalInputs[SEL_M1_AUTO_MAN].currentState},
        {"description", "Selector Motor 1 Auto/Manual"},
        {"mode", digitalInputs[SEL_M1_AUTO_MAN].currentState ? "AUTO" : "MANUAL"}
    };
    
    selectorsGroup["motor_2_auto_man"] = {
        {"state", digitalInputs[SEL_M2_AUTO_MAN].currentState},
        {"description", "Selector Motor 2 Auto/Manual"},
        {"mode", digitalInputs[SEL_M2_AUTO_MAN].currentState ? "AUTO" : "MANUAL"}
    };
    
    selectorsGroup["contactor_vfd"] = {
        {"state", digitalInputs[SEL_CONT_VARIADOR].currentState},
        {"description", "Selector Contactores/Variador"},
        {"mode", digitalInputs[SEL_CONT_VARIADOR].currentState ? "VARIADOR" : "CONTACTORES"}
    };
    
    // Grupo: Control
    JsonObject controlGroup = digitalInputsObj["control"].to<JsonObject>();
    controlGroup["emergency_stop"] = {
        {"state", digitalInputs[PARO_EMERGENCIA].currentState},
        {"description", "Paro de Emergencia"},
        {"normal_state", "NC"},
        {"active", digitalInputs[PARO_EMERGENCIA].currentState == false}
    };
    
    controlGroup["reset_alarms"] = {
        {"state", digitalInputs[RESET_ALARMAS].currentState},
        {"description", "Reset Alarmas"},
        {"normal_state", "NA"}
    };
    
    controlGroup["general_start"] = {
        {"state", digitalInputs[MARCHA_GENERAL].currentState},
        {"description", "Marcha General"},
        {"normal_state", "NA"}
    };
    
    controlGroup["general_stop"] = {
        {"state", digitalInputs[PARO_GENERAL].currentState},
        {"description", "Paro General"},
        {"normal_state", "NA"}
    };
    
    // ==============================================
    // ESTADO DE MOTORES
    // ==============================================
    
    JsonObject motorsObj = doc["motors"].to<JsonObject>();
    
    // Motor 1
    JsonObject motor1 = motorsObj["motor_1"].to<JsonObject>();
    motor1["state"] = getMotorStateString(0);
    motor1["running"] = (motor1.state == MOTOR_RUNNING);
    motor1["speed"] = motor1Speed;
    motor1["speed_percent"] = (motor1Speed / systemConfig.motorSpeedReference) * 100;
    motor1["running_hours"] = motor1.runningHours;
    motor1["start_count"] = motor1.startCount;
    motor1["operation_mode"] = getMotorModeString(0);
    motor1["control_mode"] = digitalInputs[SEL_CONT_VARIADOR].currentState ? "VFD" : "Contactor";
    
    // Motor 2
    JsonObject motor2 = motorsObj["motor_2"].to<JsonObject>();
    motor2["state"] = getMotorStateString(1);
    motor2["running"] = (motor2.state == MOTOR_RUNNING);
    motor2["speed"] = motor2Speed;
    motor2["speed_percent"] = (motor2Speed / systemConfig.motorSpeedReference) * 100;
    motor2["running_hours"] = motor2.runningHours;
    motor2["start_count"] = motor2.startCount;
    motor2["operation_mode"] = getMotorModeString(1);
    motor2["control_mode"] = digitalInputs[SEL_CONT_VARIADOR].currentState ? "VFD" : "Contactor";
    
    // Alternancia automÃ¡tica
    motorsObj["alternation"]["enabled"] = systemConfig.motorAlternationEnabled;
    motorsObj["alternation"]["time"] = systemConfig.motorAlternationTime;
    motorsObj["alternation"]["next_switch"] = calculateNextSwitchTime();
    motorsObj["alternation"]["based_on"] = "running_hours";
    
    // ==============================================
    // RED Y COMUNICACIONES
    // ==============================================
    
    JsonObject networkObj = doc["network"].to<JsonObject>();
    networkObj["ethernet"]["connected"] = ethernetConectada;
    networkObj["ethernet"]["ip"] = Ethernet.localIP().toString();
    networkObj["wifi"]["connected"] = wifiConectada;
    networkObj["wifi"]["ssid"] = cfgWifi.ssid;
    networkObj["wifi"]["ip"] = WiFi.localIP().toString();
    networkObj["wifi"]["rssi"] = WiFi.RSSI();
    networkObj["gsm"]["connected"] = gsmDisponible;
    networkObj["server"]["last_update"] = systemStatus.lastServerUpdate;
    networkObj["server"]["next_update"] = systemStatus.lastServerUpdate + externalServer.updateInterval;
    
    // ==============================================
    // ALARMAS Y EVENTOS
    // ==============================================
    
    JsonArray alarmsArray = doc["alarms"]["active"].to<JsonArray>();
    // ... agregar alarmas activas con detalles ...
    
    JsonArray eventsArray = doc["events"]["recent"].to<JsonArray>();
    // ... agregar eventos recientes ...
    
    // ==============================================
    // ESTADÃSTICAS
    // ==============================================
    
    doc["statistics"]["data_points"] = getTotalDataPoints();
    doc["statistics"]["alarms_today"] = getAlarmsToday();
    doc["statistics"]["motor_starts_today"] = getMotorStartsToday();
    doc["statistics"]["total_energy_kwh"] = calculateTotalEnergy();
    doc["statistics"]["water_pumped_m3"] = calculateWaterPumped();
    
    String output;
    serializeJsonPretty(doc, output);
    return output;
}

String generateRealTimeJsonForWebSocket() {
    /*
    JSON mÃ­nimo para actualizaciones en tiempo real via WebSocket
    Solo datos que cambian frecuentemente
    */
    JsonDocument doc;
    doc["t"] = millis();  // timestamp
    
    // Sensores que cambian rÃ¡pido
    JsonArray s = doc["s"].to<JsonArray>();
    for (int i = 0; i < 5; i++) {  // Solo presiones
        float val = getCurrentSensorValue(i);
        if (fabs(val - sensorChanges[i].lastValue) > 0.1) {
            s.add(val);
            sensorChanges[i].lastValue = val;
        } else {
            s.add(nullptr);  // null indica sin cambio
        }
    }
    
    // Nivel de agua
    doc["wl"] = getWaterLevel();
    
    // Estado motores (siempre enviar)
    doc["m1"] = motor1.state;
    doc["m2"] = motor2.state;
    doc["m1s"] = motor1Speed;
    doc["m2s"] = motor2Speed;
    
    // Alarmas activas (solo si hay cambios)
    if (checkAlarmChanges()) {
        JsonArray a = doc["a"].to<JsonArray>();
        // ... agregar alarmas ...
    }
    
    String output;
    serializeJson(doc, output);
    return output;
}

// ==============================================
// FUNCIONES PARA ENVÃO AL SERVIDOR EXTERNO
// ==============================================

void sendDataToExternalServer() {
    if (!isNetworkAvailable()) {
        Serial.println("ðŸŒ Red no disponible, posponiendo envÃ­o...");
        return;
    }
    
    String jsonData;
    
    if (externalServer.sendOnlyChanges) {
        jsonData = generateCompactJsonForServer();
        // Si no hay cambios, no enviar
        if (jsonData.length() < 50) {  // Solo metadata
            return;
        }
    } else {
        jsonData = generateFullJsonForWeb();
    }
    
    // Comprimir si estÃ¡ habilitado
    if (externalServer.enableCompression) {
        jsonData = compressJson(jsonData);
    }
    
    // Enviar con reintentos
    for (int attempt = 0; attempt < externalServer.retryCount; attempt++) {
        if (sendHttpPost(externalServer.serverUrl, jsonData)) {
            systemStatus.lastServerUpdate = millis();
            Serial.println("âœ… Datos enviados al servidor");
            logServerEvent("SUCCESS", jsonData.length());
            break;
        } else {
            Serial.printf("âŒ Intento %d fallado\n", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(1000 * (attempt + 1)));  // Backoff exponencial
            
            if (attempt == externalServer.retryCount - 1) {
                logServerEvent("FAILED", jsonData.length());
                // Guardar en SD para envÃ­o posterior
                saveDataForLater(jsonData);
            }
        }
    }
}

bool sendHttpPost(const char* url, const String& data) {
    if (WiFi.status() == WL_CONNECTED || ethernetConectada) {
        HTTPClient http;
        
        http.begin(url);
        http.setTimeout(externalServer.timeout);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-API-Key", externalServer.apiKey);
        http.addHeader("X-Device-ID", systemConfig.deviceID);
        
        if (externalServer.enableCompression) {
            http.addHeader("Content-Encoding", "gzip");
        }
        
        int httpCode = http.POST(data);
        String response = http.getString();
        http.end();
        
        if (httpCode == 200 || httpCode == 201) {
            return true;
        } else {
            Serial.printf("HTTP Error: %d, Response: %s\n", httpCode, response.c_str());
            return false;
        }
    }
    
    return false;
}

void saveDataForLater(const String& data) {
    // Guardar datos en SD para enviar cuando haya conexiÃ³n
    char filename[32];
    snprintf(filename, sizeof(filename), "/pending/data_%lu.json", millis());
    
    File file = SD.open(filename, FILE_WRITE);
    if (file) {
        file.println(data);
        file.close();
        Serial.printf("ðŸ’¾ Datos guardados para envÃ­o posterior: %s\n", filename);
    }
}

void sendPendingData() {
    // Enviar datos pendientes almacenados en SD
    File dir = SD.open("/pending");
    if (!dir) return;
    
    File file = dir.openNextFile();
    while (file) {
        if (!file.isDirectory()) {
            String data = file.readString();
            file.close();
            
            if (sendHttpPost(externalServer.serverUrl, data)) {
                // Eliminar archivo enviado
                String path = "/pending/" + String(file.name());
                SD.remove(path.c_str());
                Serial.printf("âœ… Datos pendientes enviados: %s\n", file.name());
            }
        }
        file = dir.openNextFile();
    }
    
    dir.close();
}

// ==============================================
// SISTEMA DE DETECCIÃ“N DE CAMBIOS
// ==============================================

void initChangeDetection() {
    for (int i = 0; i < 16; i++) {
        sensorChanges[i].lastValue = 0.0;
        sensorChanges[i].lastSendTime = 0;
        sensorChanges[i].needsUpdate = false;
        sensorChanges[i].changeThreshold = externalServer.changeThreshold;
    }
    
    for (int i = 0; i < 16; i++) {
        digitalChanges[i] = false;
    }
    
    memset(lastMotorStatus, 0, sizeof(lastMotorStatus));
}

void checkSensorChanges() {
    uint32_t now = millis();
    
    for (int i = 0; i < 16; i++) {
        float currentValue = getCurrentSensorValue(i);
        float lastValue = sensorChanges[i].lastValue;
        
        // Calcular cambio porcentual
        float range = analogConfigs[i].maxVoltage - analogConfigs[i].minVoltage;
        if (range > 0) {
            float changePercent = fabs(currentValue - lastValue) / range * 100.0;
            
            if (changePercent > sensorChanges[i].changeThreshold) {
                sensorChanges[i].needsUpdate = true;
                sensorChanges[i].lastValue = currentValue;
                sensorChanges[i].lastSendTime = now;
            }
        }
        
        // Forzar envÃ­o periÃ³dico (cada 5 minutos)
        if (now - sensorChanges[i].lastSendTime > 300000) {
            sensorChanges[i].needsUpdate = true;
            sensorChanges[i].lastSendTime = now;
        }
    }
}

void checkDigitalChanges() {
    for (int i = 0; i < 16; i++) {
        bool currentState = digitalInputs[i].currentState;
        bool lastState = digitalInputs[i].lastState;
        
        if (currentState != lastState) {
            digitalChanges[i] = true;
        }
    }
}

void checkMotorChanges() {
    // Comparar estado actual con anterior
    if (memcmp(&motor1, &lastMotorStatus[0], sizeof(MotorStatus)) != 0 ||
        memcmp(&motor2, &lastMotorStatus[1], sizeof(MotorStatus)) != 0) {
        
        // Hay cambios en los motores
        memcpy(&lastMotorStatus[0], &motor1, sizeof(MotorStatus));
        memcpy(&lastMotorStatus[1], &motor2, sizeof(MotorStatus));
        
        // Marcar para envÃ­o inmediato
        systemStatus.lastServerUpdate = 0;  // Forzar envÃ­o
    }
}

// ==============================================
// SERVIDOR WEB CON API REST
// ==============================================

WebServer webServer(80);

void initWebServer() {
    Serial.println("ðŸŒ Inicializando servidor web...");
    
    // Servir archivos estÃ¡ticos
    webServer.serveStatic("/", SD, "/www/").setDefaultFile("index.html");
    
    // API REST para datos
    webServer.on("/api/status", HTTP_GET, []() {
        webServer.send(200, "application/json", generateFullJsonForWeb());
    });
    
    webServer.on("/api/realtime", HTTP_GET, []() {
        webServer.send(200, "application/json", generateRealTimeJsonForWebSocket());
    });
    
    webServer.on("/api/sensors", HTTP_GET, []() {
        JsonDocument doc;
        JsonArray sensors = doc.to<JsonArray>();
        
        for (int i = 0; i < 16; i++) {
            if (analogConfigs[i].sensorType[0] != '\0') {
                JsonObject sensor = sensors.add<JsonObject>();
                sensor["id"] = i;
                sensor["name"] = analogConfigs[i].sensorType;
                sensor["value"] = getCurrentSensorValue(i);
                sensor["units"] = analogConfigs[i].units;
            }
        }
        
        String output;
        serializeJson(doc, output);
        webServer.send(200, "application/json", output);
    });
    
    webServer.on("/api/digital", HTTP_GET, []() {
        JsonDocument doc;
        
        for (int i = 0; i < 16; i++) {
            String key = String("d") + i;
            doc[key] = digitalInputs[i].currentState;
        }
        
        String output;
        serializeJson(doc, output);
        webServer.send(200, "application/json", output);
    });
    
    webServer.on("/api/motors", HTTP_GET, []() {
        JsonDocument doc;
        
        doc["motor1"]["state"] = getMotorStateString(0);
        doc["motor1"]["speed"] = motor1Speed;
        doc["motor1"]["running"] = (motor1.state == MOTOR_RUNNING);
        
        doc["motor2"]["state"] = getMotorStateString(1);
        doc["motor2"]["speed"] = motor2Speed;
        doc["motor2"]["running"] = (motor2.state == MOTOR_RUNNING);
        
        String output;
        serializeJson(doc, output);
        webServer.send(200, "application/json", output);
    });
    
    webServer.on("/api/control/motor1/start", HTTP_POST, []() {
        if (digitalInputs[PARO_EMERGENCIA].currentState) {  // Paro activo
            webServer.send(403, "application/json", "{\"error\":\"Emergency stop active\"}");
            return;
        }
        
        MotorCommand cmd;
        cmd.motorId = 0;
        cmd.command = MOTOR_START;
        xQueueSend(xMotorQueue, &cmd, 0);
        
        webServer.send(200, "application/json", "{\"status\":\"Motor 1 starting\"}");
    });
    
    webServer.on("/api/control/motor1/stop", HTTP_POST, []() {
        MotorCommand cmd;
        cmd.motorId = 0;
        cmd.command = MOTOR_STOP;
        xQueueSend(xMotorQueue, &cmd, 0);
        
        webServer.send(200, "application/json", "{\"status\":\"Motor 1 stopping\"}");
    });
    
    // ConfiguraciÃ³n
    webServer.on("/api/config", HTTP_GET, []() {
        File file = SD.open(CONFIG_FILE, FILE_READ);
        if (file) {
            webServer.streamFile(file, "application/json");
            file.close();
        } else {
            webServer.send(404, "text/plain", "Config file not found");
        }
    });
    
    // Logs
    webServer.on("/api/logs", HTTP_GET, []() {
        File file = SD.open(LOG_FILE, FILE_READ);
        if (file) {
            webServer.streamFile(file, "text/plain");
            file.close();
        } else {
            webServer.send(404, "text/plain", "Log file not found");
        }
    });
    
    // PÃ¡gina de diagnÃ³stico
    webServer.on("/api/diagnostic", HTTP_GET, []() {
        JsonDocument doc;
        
        doc["system"]["free_heap"] = esp_get_free_heap_size();
        doc["system"]["min_heap"] = esp_get_minimum_free_heap_size();
        doc["system"]["uptime"] = millis() / 1000;
        doc["system"]["reset_reason"] = esp_reset_reason();
        
        doc["network"]["ethernet"] = ethernetConectada;
        doc["network"]["wifi"] = wifiConectada;
        doc["network"]["gsm"] = gsmDisponible;
        
        doc["storage"]["sd_total"] = SD.cardSize();
        doc["storage"]["sd_used"] = calculateUsedSpace();
        doc["storage"]["sd_free"] = SD.cardSize() - calculateUsedSpace();
        
        doc["tasks"] = getTaskStatusJson();
        
        String output;
        serializeJsonPretty(doc, output);
        webServer.send(200, "application/json", output);
    });
    
    // Iniciar servidor
    webServer.begin();
    Serial.println("âœ… Servidor web iniciado en puerto 80");
}

void handleWebServer() {
    webServer.handleClient();
}

// ==============================================
// TAREAS FreeRTOS PARA JSON Y COMUNICACIONES
// ==============================================

void vTaskJsonServerSender(void *pvParameters) {
    Serial.println("ðŸ“¤ Tarea JsonServerSender iniciada");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // 1. Verificar cambios
        checkSensorChanges();
        checkDigitalChanges();
        checkMotorChanges();
        
        // 2. Enviar datos al servidor
        sendDataToExternalServer();
        
        // 3. Enviar datos pendientes
        sendPendingData();
        
        // 4. Esperar siguiente ciclo
        vTaskDelayUntil(&xLastWakeTime, 
                       pdMS_TO_TICKS(externalServer.updateInterval));
    }
}

void vTaskWebServerHandler(void *pvParameters) {
    Serial.println("ðŸŒ Tarea WebServerHandler iniciada");
    
    initWebServer();
    
    while (1) {
        handleWebServer();
        vTaskDelay(pdMS_TO_TICKS(10));  // PequeÃ±o delay
    }
}

void vTaskJsonDataLogger(void *pvParameters) {
    Serial.println("ðŸ“ Tarea JsonDataLogger iniciada");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(60000);  // 1 minuto
    
    while (1) {
        // Registrar datos histÃ³ricos en SD
        logHistoricalData();
        
        // Rotar logs si es necesario
        rotateLogs();
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ==============================================
// FUNCIONES AUXILIARES
// ==============================================

String getOperationModeString() {
    if (digitalInputs[PARO_EMERGENCIA].currentState == false) {
        return "EMERGENCY_STOP";
    }
    
    bool motor1Auto = digitalInputs[SEL_M1_AUTO_MAN].currentState;
    bool motor2Auto = digitalInputs[SEL_M2_AUTO_MAN].currentState;
    
    if (motor1Auto && motor2Auto) return "FULL_AUTO";
    if (!motor1Auto && !motor2Auto) return "FULL_MANUAL";
    if (motor1Auto && !motor2Auto) return "MIXED_M1_AUTO";
    if (!motor1Auto && motor2Auto) return "MIXED_M2_AUTO";
    
    return "UNKNOWN";
}

String getMotorStateString(uint8_t motorId) {
    MotorStatus* motor = (motorId == 0) ? &motor1 : &motor2;
    
    switch(motor->state) {
        case MOTOR_OFF: return "OFF";
        case MOTOR_STARTING: return "STARTING";
        case MOTOR_RUNNING: return "RUNNING";
        case MOTOR_STOPPING: return "STOPPING";
        case MOTOR_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

String getMotorModeString(uint8_t motorId) {
    if (motorId == 0) {
        return digitalInputs[SEL_M1_AUTO_MAN].currentState ? "AUTO" : "MANUAL";
    } else {
        return digitalInputs[SEL_M2_AUTO_MAN].currentState ? "AUTO" : "MANUAL";
    }
}

String getFormattedTime() {
    DateTime now = rtc.now();
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    return String(buffer);
}

String formatUptime(uint32_t seconds) {
    uint32_t days = seconds / 86400;
    uint32_t hours = (seconds % 86400) / 3600;
    uint32_t minutes = (seconds % 3600) / 60;
    uint32_t secs = seconds % 60;
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%lu dias, %02lu:%02lu:%02lu",
             days, hours, minutes, secs);
    return String(buffer);
}

String getSensorStatus(uint8_t channel) {
    InputFault fault = checkInputFault(channel);
    
    switch(fault) {
        case INPUT_OK: return "OK";
        case INPUT_UNDER_RANGE: return "UNDER_RANGE";
        case INPUT_OVER_RANGE: return "OVER_RANGE";
        case INPUT_OPEN_CIRCUIT: return "OPEN_CIRCUIT";
        case INPUT_SHORT_CIRCUIT: return "SHORT_CIRCUIT";
        case INPUT_NOISE: return "NOISY";
        case INPUT_DRIFT: return "DRIFTING";
        default: return "UNKNOWN";
    }
}

void logServerEvent(const char* status, size_t dataSize) {
    JsonDocument doc;
    doc["timestamp"] = millis();
    doc["status"] = status;
    doc["data_size"] = dataSize;
    doc["server_url"] = externalServer.serverUrl;
    doc["device_id"] = systemConfig.deviceID;
    
    File file = SD.open("/logs/server_events.json", FILE_APPEND);
    if (file) {
        serializeJson(doc, file);
        file.println();
        file.close();
    }
}

String getTaskStatusJson() {
    JsonDocument doc;
    
    // InformaciÃ³n de tareas FreeRTOS
    doc["total_tasks"] = uxTaskGetNumberOfTasks();
    
    // Obtener informaciÃ³n de tareas principales
    JsonArray tasks = doc["tasks"].to<JsonArray>();
    
    addTaskInfo(tasks, "SystemMonitor", xSystemMonitorHandle);
    addTaskInfo(tasks, "SensorReader", xSensorReaderHandle);
    addTaskInfo(tasks, "MotorControl", xMotorControlHandle);
    addTaskInfo(tasks, "JsonServerSender", xTaskGetHandle("JsonServerSender"));
    addTaskInfo(tasks, "WebServerHandler", xTaskGetHandle("WebServerHandler"));
    addTaskInfo(tasks, "JsonDataLogger", xTaskGetHandle("JsonDataLogger"));
    
    String output;
    serializeJson(doc, output);
    return output;
}

void addTaskInfo(JsonArray& array, const char* name, TaskHandle_t handle) {
    if (handle != NULL) {
        JsonObject task = array.add<JsonObject>();
        task["name"] = name;
        task["state"] = pcTaskGetStateName(eTaskGetState(handle));
        task["priority"] = uxTaskPriorityGet(handle);
        task["stack_high_water"] = uxTaskGetStackHighWaterMark(handle);
    }
}

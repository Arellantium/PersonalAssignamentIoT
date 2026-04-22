#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include "arduinoFFT.h"

// --- LIBRERIA RADIOLIB (Per chip SX1262 - Heltec V3) ---
#include <RadioLib.h>

// --- CONFIGURAZIONE RETE WI-FI ---
const char* ssid = "Alessandro";
const char* password = "";
const char* mqtt_server = "172.20.10.2";

// =========================================================
//  PARAMETRI DI TEST PER IL REPORT 
// =========================================================
float SIGNAL_FREQ_1 = 3.0;     
float SIGNAL_FREQ_2 = 5.0;     
int   ANOMALY_PROB_PCT = 15;    
#define HAMPEL_WINDOW_SIZE 7   
// =========================================================

// --- INIZIALIZZAZIONE HARDWARE RADIO (Heltec V3) ---
// Pin: NSS=8, DIO1=14, NRST=12, BUSY=13
SX1262 radio = new Module(8, 14, 12, 13);
LoRaWANNode node(&radio, &EU868);

float samplingFreq = 1000.0; 
const int windowSec = 5;

// --- CODE DI COMUNICAZIONE ---
QueueHandle_t mqttQueue;
QueueHandle_t loraQueue; 
WiFiClient espClient;
PubSubClient client(espClient);

#define MAX_SAMPLES 4096
float vReal[MAX_SAMPLES];
float vImag[MAX_SAMPLES];
float vRaw[MAX_SAMPLES]; 
bool trueAnomalies[MAX_SAMPLES];
float cleanData[MAX_SAMPLES];

float generateGaussianNoise(float mu, float sigma) {
    float u1 = (float)random(1, 10000) / 10000.0;
    float u2 = (float)random(1, 10000) / 10000.0;
    float z0 = sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
    return z0 * sigma + mu;
}

void applyZScore(float* data, int size, bool* trueAnomalies, int& tp, int& fp) {
    tp = 0; fp = 0;
    float sum = 0;
    for (int i = 0; i < size; i++) sum += data[i];
    float mean = sum / size;
    float varianceSum = 0;
    for (int i = 0; i < size; i++) varianceSum += pow(data[i] - mean, 2);
    float stdDev = sqrt(varianceSum / size);
    for (int i = 0; i < size; i++) {
        if (stdDev > 0) {
            float zScore = abs(data[i] - mean) / stdDev;
            if (zScore > 3.0) {
                if (trueAnomalies[i]) tp++;
                else fp++;
            }
        }
    }
}

void applyHampel(float* data, int size, bool* trueAnomalies, int& tp, int& fp) {
    tp = 0; fp = 0;
    int halfWin = HAMPEL_WINDOW_SIZE / 2;
    float threshold = 3.0; 
    const float scaleFactor = 1.4826; 
    for(int i=0; i<size; i++) cleanData[i] = data[i];
    for (int i = halfWin; i < size - halfWin; i++) {
        float window[HAMPEL_WINDOW_SIZE]; 
        for (int j = 0; j < HAMPEL_WINDOW_SIZE; j++) window[j] = data[i - halfWin + j];
        for (int a = 1; a < HAMPEL_WINDOW_SIZE; a++) {
            float key = window[a];
            int b = a - 1;
            while (b >= 0 && window[b] > key) { window[b + 1] = window[b]; b--; }
            window[b + 1] = key;
        }
        float median = window[halfWin]; 
        float deviations[HAMPEL_WINDOW_SIZE];
        for(int j=0; j<HAMPEL_WINDOW_SIZE; j++) deviations[j] = abs(data[i - halfWin + j] - median);
        for (int a = 1; a < HAMPEL_WINDOW_SIZE; a++) {
            float key = deviations[a];
            int b = a - 1;
            while (b >= 0 && deviations[b] > key) { deviations[b + 1] = deviations[b]; b--; }
            deviations[b + 1] = key;
        }
        float mad = deviations[halfWin];
        if (abs(data[i] - median) > threshold * mad * scaleFactor) {
            if (trueAnomalies[i]) tp++;
            else fp++;
            cleanData[i] = median; 
        }
    }
    for(int i=0; i<size; i++) data[i] = cleanData[i];
}

// --- TASK LORAWAN (RADIOLIB) ---
void vTaskLoRa(void * pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000));
    Serial.println("\n[LoRa] Inizializzazione Chip SX1262 (Heltec V3)...");
    
    int state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LoRa] Errore hardware radio, codice: %d\n", state);
        while (true) vTaskDelay(10000); // Si ferma qui se l'hardware fallisce
    }

    Serial.println("[LoRa] Radio pronta. Tentativo di JOIN OTAA su TTN...");

    // Le tue chiavi esatte da The Things Network (Formato MSB per RadioLib)
    uint64_t joinEUI = 0x0000000000000000;
    uint64_t devEUI  = 0x70B3D57ED0076FC7;
    uint8_t appKey[] = { 0xDF, 0xF8, 0x63, 0xAC, 0xAC, 0xC5, 0x59, 0x8A, 0xE5, 0xC5, 0x08, 0x94, 0x62, 0x2D, 0x2D, 0x08 };
    uint8_t nwkKey[] = { 0xDF, 0xF8, 0x63, 0xAC, 0xAC, 0xC5, 0x59, 0x8A, 0xE5, 0xC5, 0x08, 0x94, 0x62, 0x2D, 0x2D, 0x08 }; // In v1.0.3 coincide con appKey

    // Avvia la connessione
    // 1. Carica le chiavi in memoria (Senza 'state)
    node.beginOTAA(joinEUI, devEUI, nwkKey, appKey); 

    Serial.println("[LoRa] Tentativo di JOIN OTAA...");

    // 2. Forza un Duty Cycle basso per il test e prova l'attivazione
    // Nota: activateOTAA() in RadioLib gestisce i canali standard automaticamente per EU868
    state = node.activateOTAA(); 

    if (state == RADIOLIB_ERR_NONE) {
        Serial.println("[LoRa] JOIN EFFETTUATO CON SUCCESSO!");
    } else {
        // Se vedi ancora un errore, stampiamo il codice per capire se è un timeout (-1117) o altro
        Serial.printf("[LoRa] JOIN FALLITO. Codice errore: %d\n", state);
        Serial.println("[LoRa] Suggerimento: Verifica che il Gateway sia nel raggio d'azione o sposta l'antenna.");
    }

    float loraData;
    while (true) {
        if (xQueueReceive(loraQueue, &loraData, portMAX_DELAY) == pdPASS) {
            // Converte il float (es. 3.14) in int16_t (314) per risparmiare banda
            int16_t payload = (int16_t)(loraData * 100);
            byte loraPayload[2];
            loraPayload[0] = highByte(payload);
            loraPayload[1] = lowByte(payload);

            Serial.printf("\n📡 [LoRa] Inviando il valore %d a The Things Network...\n", payload);
            
            // Invia il pacchetto
            state = node.sendReceive(loraPayload, 2);

            if (state == RADIOLIB_ERR_NONE || state == RADIOLIB_LORAWAN_NONCES_DISCARDED) {
                Serial.println(" [LoRa] PACCHETTO CONSEGNATO AL CLOUD!");
            } else {
                Serial.printf(" [LoRa] Errore di invio radio. Codice: %d\n", state);
            }
        }
    }
}

// --- TASK PROCESSING E FFT ---
void vTaskProcess(void * pvParameters) {
    while (true) {
        int expected = (int)(samplingFreq * windowSec);
        int fftSamples = 1;
        while (fftSamples * 2 <= expected) fftSamples *= 2;
        if (fftSamples > MAX_SAMPLES) fftSamples = MAX_SAMPLES;

        int actualAnomalies = 0;
        int evaluableAnomaliesHampel = 0;

        for (int i = 0; i < expected; i++) {
            float t = (float)i / samplingFreq;
            float sample = 2.0 * sin(2.0 * PI * SIGNAL_FREQ_1 * t) + 4.0 * sin(2.0 * PI * SIGNAL_FREQ_2 * t);
            sample += generateGaussianNoise(0.0, 0.2); 

            bool isAnomaly = false;
            if (random(0, 100) < ANOMALY_PROB_PCT) { 
                float spikeMag = (float)random(500, 1500) / 100.0; 
                sample += (random(0, 2) == 0 ? spikeMag : -spikeMag); 
                isAnomaly = true;
                if (i < fftSamples) {
                    actualAnomalies++;
                    if (i >= (HAMPEL_WINDOW_SIZE/2) && i < fftSamples - (HAMPEL_WINDOW_SIZE/2)) {
                        evaluableAnomaliesHampel++;
                    }
                }
            }
            if (i < fftSamples) {
                vRaw[i] = sample;  
                vReal[i] = sample; 
                trueAnomalies[i] = isAnomaly;
            }
            delayMicroseconds(1000000 / samplingFreq); 
            if (i % 500 == 0) vTaskDelay(1);
        }

        int normalSamplesZ = fftSamples - actualAnomalies;
        int normalSamplesH = (fftSamples - HAMPEL_WINDOW_SIZE) - evaluableAnomaliesHampel;

        int tpZ=0, fpZ=0;
        uint32_t t1 = micros();
        applyZScore(vRaw, fftSamples, trueAnomalies, tpZ, fpZ);
        uint32_t timeZScore = micros() - t1;

        int tpH=0, fpH=0;
        uint32_t t2 = micros();
        applyHampel(vReal, fftSamples, trueAnomalies, tpH, fpH);
        uint32_t timeHampel = micros() - t2;

        float tprZScore = (actualAnomalies > 0) ? ((float)tpZ / actualAnomalies) * 100.0 : 0;
        float fprZScore = (normalSamplesZ > 0) ? ((float)fpZ / normalSamplesZ) * 100.0 : 0;
        float tprHampel = (evaluableAnomaliesHampel > 0) ? ((float)tpH / evaluableAnomaliesHampel) * 100.0 : 0;
        float fprHampel = (normalSamplesH > 0) ? ((float)fpH / normalSamplesH) * 100.0 : 0;

        ArduinoFFT<float> FFT = ArduinoFFT<float>(NULL, NULL, fftSamples, samplingFreq);
        for(int i=0; i<fftSamples; i++) vImag[i] = 0.0;
        FFT.setArrays(vRaw, vImag);
        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(FFT_FORWARD);
        FFT.complexToMagnitude();
        float peakDirty = FFT.majorPeak();

        for(int i=0; i<fftSamples; i++) vImag[i] = 0.0;
        FFT.setArrays(vReal, vImag);
        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(FFT_FORWARD);
        FFT.complexToMagnitude();
        float peakClean = FFT.majorPeak();

        Serial.println("\n==================================================");
        Serial.printf("Z-SCORE -> TPR: %6.2f%% | FPR: %6.2f%% | Tempo: %u us\n", tprZScore, fprZScore, timeZScore);
        Serial.printf("HAMPEL  -> TPR: %6.2f%% | FPR: %6.2f%% | Tempo: %u us\n", tprHampel, fprHampel, timeHampel);
        Serial.println("==================================================\n");

        if (samplingFreq > 100) samplingFreq = peakClean * 2.0;

        float avgVal = 0; 
        for(int i=0; i<fftSamples; i++) avgVal += vReal[i];
        avgVal /= fftSamples;
        
        xQueueSend(mqttQueue, &avgVal, 0);
        xQueueSend(loraQueue, &avgVal, 0);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// --- CALCOLO LATENZA (4-TIMESTAMP) ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    uint32_t t4 = millis(); 
    String message;
    for (int i = 0; i < length; i++) message += (char)payload[i];
    JsonDocument doc;
    deserializeJson(doc, message);
    uint32_t t1 = doc["t1"]; 
    uint32_t t2 = doc["t2"]; 
    uint32_t t3 = doc["t3"]; 
    uint32_t rtt = (t4 - t1) - (t3 - t2);
    Serial.printf("🌐 [LATENZA MQTT] E2E: %u ms | RTT: %u ms\n", rtt / 2, rtt);
}

// --- TASK MQTT ---
void vTaskMQTT(void * pvParameters) {
    float val;
    client.setCallback(mqttCallback); 
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.begin(ssid, password);
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (!client.connected()) {
            if (client.connect("ESP32_Client")) {
                client.subscribe("iot/sensor/ack"); 
            } else {
                vTaskDelay(pdMS_TO_TICKS(5000));
                continue;
            }
        }
        if (xQueueReceive(mqttQueue, &val, portMAX_DELAY) == pdPASS) {
            JsonDocument doc;
            doc["avg"] = val;
            doc["t1"] = millis(); 
            char buffer[256];
            serializeJson(doc, buffer);
            client.publish("iot/sensor/data", buffer);
        }
        client.loop(); 
    }
}

void setup() {
    Serial.begin(115200);
    delay(3000);
    
    mqttQueue = xQueueCreate(10, sizeof(float));
    loraQueue = xQueueCreate(10, sizeof(float)); 
    
    WiFi.begin(ssid, password);
    client.setServer(mqtt_server, 1883);

    xTaskCreatePinnedToCore(vTaskProcess, "Process", 8192, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vTaskMQTT, "MQTT", 8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(vTaskLoRa, "LoRaWAN", 8192, NULL, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }
